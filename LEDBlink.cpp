#include "MKL27Z644.h"

void Delay()
{
	int i;
	for (i = 0; i < 1000000; i++)
		asm("nop");
}

int main()
{
	SIM_BASE_PTR->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_BASE_PTR->PCR[18] = PORT_PCR_MUX(1);

	GPIOB_PDDR = 1 << 18;

	for(;;)
	{
		GPIOB_PSOR = 1 << 18;
		Delay();
		GPIOB_PCOR = 1 << 18;
		Delay();
	}

	return 0;
}