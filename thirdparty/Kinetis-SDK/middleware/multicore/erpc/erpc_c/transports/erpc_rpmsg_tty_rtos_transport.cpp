/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "erpc_rpmsg_tty_rtos_transport.h"
#include "erpc_config_internal.h"
#include "erpc_framed_transport.h"
#include "rpmsg_ns.h"
#include <cassert>

using namespace erpc;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
uint8_t RPMsgBaseTransport::s_initialized = 0;
struct rpmsg_lite_instance *RPMsgBaseTransport::s_rpmsg;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

RPMsgTTYRTOSTransport::RPMsgTTYRTOSTransport(void)
: RPMsgBaseTransport()
, m_dst_addr(0)
, m_rpmsg_queue(NULL)
, m_rpmsg_ept(NULL)
, m_crcImpl(NULL)
{
}

RPMsgTTYRTOSTransport::~RPMsgTTYRTOSTransport(void)
{
    rpmsg_lite_deinit(s_rpmsg);
    s_initialized = 0;
}

void RPMsgTTYRTOSTransport::setCrc16(Crc16 *crcImpl)
{
    assert(crcImpl);
    m_crcImpl = crcImpl;
}

erpc_status_t RPMsgTTYRTOSTransport::init(unsigned long src_addr, unsigned long dst_addr, void *base_address,
                                          unsigned long length, int rpmsg_link_id)
{
    if (!s_initialized)
    {
        s_rpmsg = rpmsg_lite_master_init(base_address, length, rpmsg_link_id, RL_NO_FLAGS);
        if (!s_rpmsg)
        {
            return kErpcStatus_InitFailed;
        }
        s_initialized = 1;
    }

    m_rpmsg_queue = rpmsg_queue_create(s_rpmsg);
    if (!m_rpmsg_queue)
    {
        return kErpcStatus_InitFailed;
    }

    m_rpmsg_ept = rpmsg_lite_create_ept(s_rpmsg, src_addr, rpmsg_queue_rx_cb, m_rpmsg_queue);

    m_dst_addr = dst_addr;
    return m_rpmsg_ept == RL_NULL ? kErpcStatus_InitFailed : kErpcStatus_Success;
}

erpc_status_t RPMsgTTYRTOSTransport::init(unsigned long src_addr, unsigned long dst_addr, void *base_address,
                                          int rpmsg_link_id, void (*ready_cb)(void), char *nameservice_name)
{
    if (!s_initialized)
    {
        s_rpmsg = rpmsg_lite_remote_init(base_address, rpmsg_link_id, RL_NO_FLAGS);
        if (!s_rpmsg)
        {
            return kErpcStatus_InitFailed;
        }

        /* Signal the other core we are ready */
        if (ready_cb != NULL)
        {
            ready_cb();
        }

        while (!rpmsg_lite_is_link_up(s_rpmsg))
        {
        }

        s_initialized = 1;
    }

    m_rpmsg_queue = rpmsg_queue_create(s_rpmsg);
    if (!m_rpmsg_queue)
    {
        return kErpcStatus_InitFailed;
    }
    m_rpmsg_ept = rpmsg_lite_create_ept(s_rpmsg, src_addr, rpmsg_queue_rx_cb, m_rpmsg_queue);

    if (nameservice_name)
    {
        if (RL_SUCCESS != rpmsg_ns_announce(s_rpmsg, m_rpmsg_ept, nameservice_name, RL_NS_CREATE))
        {
            return kErpcStatus_InitFailed;
        }
    }

    m_dst_addr = dst_addr;
    return m_rpmsg_ept == RL_NULL ? kErpcStatus_InitFailed : kErpcStatus_Success;
}

erpc_status_t RPMsgTTYRTOSTransport::receive(MessageBuffer *message)
{
    assert(m_crcImpl && "Uninitialized Crc16 object.");
    FramedTransport::Header h;
    char *buf = NULL;
    int length = 0;

    int ret_val = rpmsg_queue_recv_nocopy(s_rpmsg, m_rpmsg_queue, &m_dst_addr, &buf, &length, RL_BLOCK);
    assert(buf);

    memcpy((uint8_t *)&h, buf, sizeof(h));

    message->set(&((uint8_t *)buf)[sizeof(h)], length - sizeof(h));

    // Verify CRC.
    uint16_t computedCrc = m_crcImpl->computeCRC16(&((uint8_t *)buf)[sizeof(h)], h.m_messageSize);
    if (computedCrc != h.m_crc)
    {
        return kErpcStatus_CrcCheckFailed;
    }

    message->setUsed(h.m_messageSize);
    return ret_val != RL_SUCCESS ? kErpcStatus_ReceiveFailed : kErpcStatus_Success;
}

erpc_status_t RPMsgTTYRTOSTransport::send(MessageBuffer *message)
{
    assert(m_crcImpl && "Uninitialized Crc16 object.");
    FramedTransport::Header h;
    uint8_t *buf = message->get();
    uint32_t length = message->getLength();
    uint32_t used = message->getUsed();
    message->set(NULL, 0);

    h.m_crc = m_crcImpl->computeCRC16(buf, used);
    h.m_messageSize = used;

    memcpy(buf - sizeof(h), (uint8_t *)&h, sizeof(h));

    int ret_val = rpmsg_lite_send_nocopy(s_rpmsg, m_rpmsg_ept, m_dst_addr, buf - sizeof(h), used + sizeof(h));
    if (ret_val == RL_SUCCESS)
    {
        return kErpcStatus_Success;
    }
    message->set(buf, length);
    message->setUsed(used);
    return kErpcStatus_SendFailed;
}
