/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



typedef enum{
	flag_EnableLED = 0 ,
	
	flag_DEFAULT	
}flag_Index;

#define HIBYTE(v1)              					((uint8_t)((v1)>>8))                      //v1 is UINT16
#define LOBYTE(v1)              					((uint8_t)((v1)&0xFF))

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

#define GPIO_P0_TO_P15              				(0xFFFF)

uint32_t conter_tick = 0;
//uint8_t gpioState = DISABLE;
uint8_t buttonState;
uint8_t lastButtonState = DISABLE;
uint32_t lastDebounceTime = 0;
uint32_t debounceDelay = 50;

void convertDecToBin(int n)
{
    int k = 0;
    unsigned char *p = (unsigned char*)&n;
    int val2 = 0;
    int i = 0;
    for(k = 0; k <= 1; k++)
    {
        val2 = *(p+k);
        for (i = 7; i >= 0; i--)
        {
            if(val2 & (1 << i))
                printf("1");
            else
                printf("0");
        }
        printf(" ");
    }
}


void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void SYS_Disable_AnalogPORCircuit(void)
{
    SYS->PORDISAN = 0x5AA5;
}


void EntryLowPower(void)
{
	printf("%s\r\n" , __FUNCTION__);

	set_flag(flag_EnableLED , ENABLE);

	#if 1
	CLK_Idle();
	#else
    UART_WAIT_TX_EMPTY(UART0);
	
    /* Configure all GPIO as Quasi-bidirectional Mode*/
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* LVR must be enabled and the POR will be enabled automatically */
    SYS_ENABLE_LVR();
    /* Turn off internal analog POR circuit */
    SYS_Disable_AnalogPORCircuit();
    /* Disable Power-on Reset */
    SYS_DISABLE_POR();

    UART_WAIT_TX_EMPTY(UART0);
	CLK_PowerDown();

	#endif



}

void GPCDEF_IRQHandler(void)
{
    volatile uint32_t temp;

    if(GPIO_GET_INT_FLAG(PC, BIT3))
    {
        GPIO_CLR_INT_FLAG(PC, BIT3);
		set_flag(flag_EnableLED , DISABLE);				
        printf("PC.3 wake up\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PC->INTSRC;
        PC->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
}

void GPABGH_IRQHandler(void)
{
    volatile uint32_t temp;

    if(GPIO_GET_INT_FLAG(PA, BIT7))
    {
        GPIO_CLR_INT_FLAG(PA, BIT7);
		set_flag(flag_EnableLED , DISABLE);		
        printf("PA.7 wake up\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PA->INTSRC;
        PA->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
}


void GPIO_Power_Init (void)		// PB3 , to entry low power mode
{
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);

//    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
//    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_8192);
//    GPIO_ENABLE_DEBOUNCE(PB, BIT3);	
}

void GPIO_Wakeup_Init (void)		// PA7/PC3 , to wake up MCU
{

    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 7, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_PAPBPGPH_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_2048);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    GPIO_SetMode(PC, BIT3, GPIO_MODE_INPUT);
    GPIO_EnableInt(PC, 3, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPIO_PCPDPEPF_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_2048);
    GPIO_ENABLE_DEBOUNCE(PC, BIT3);
	
}


void Software_Debounce(void)
{
    uint8_t reading = PB3;		// button

    if (reading != lastButtonState) 
	{
        // reset the debouncing timer
        lastDebounceTime = get_tick();
    }

    if ((get_tick() - lastDebounceTime) > debounceDelay)
	{
        if (reading != buttonState) 
		{ 
            buttonState = reading;

            if (buttonState == ENABLE) 
			{
				EntryLowPower();
            }
        }
    }

    lastButtonState = reading;

}

void TMR3_IRQHandler(void)
{
	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);

		tick_counter();

		if (is_flag_set(flag_EnableLED))
		{
			if (CNT++ >= 1000)
			{		
				CNT = 0;
				PB14 ^= 1;
			}
		}
		else
		{	
			if (CNT++ >= 250)
			{		
				CNT = 0;
	        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
				PB14 ^= 1;

			}
		}
    }
}


void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}


void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case 'Z':
			case 'z':				
				NVIC_SystemReset();
				break;				
		}
	}
}

void UART02_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART02_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);


    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_LIRC, 0);
//   CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    SYS_Init();

    UART0_Init();

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
	GPIO_Power_Init();
	GPIO_Wakeup_Init();

	TIMER3_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
		Software_Debounce();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
