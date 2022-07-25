/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "NuMicro.h"

#include "project_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

uint16_t za = 0;
uint16_t zb = 0;
uint16_t zc = 0;
uint16_t zx = 0;

#define SNUM        32       /* recorded number of lastest samples */
uint32_t   adc_val[SNUM] = {0};
uint32_t   val_sum = 0;
int        oldest = 0;

/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/


void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay_ms(uint32_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

uint16_t get_adc_bg_val(void)
{
    uint16_t  val;    

    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);    

	ADC_START_CONV(ADC);		

    while (ADC_GET_INT_FLAG(ADC, ADC_ADF_INT)==0);
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);    

    __WFI();
    while(ADC_IS_DATA_VALID(ADC, 29) == 0);     

    val = ADC_GET_CONVERSION_DATA(ADC, 29);

    // printf("0X%4X:Band-gap voltage is %dmV if Reference voltage is 3.3V\n", val ,(3300*val)/4095);

	return (val);
}

int adc_trng_gen_bit(void)
{
    uint32_t   new_val, average;
    int        ret_val;

    new_val = get_adc_bg_val();

    // average = (val_sum / SNUM);   /* sum divided by 32 */
    average = (val_sum >> 5 );   /* sum divided by 32 */

    if (average >= new_val)
        ret_val = 1;
    else
        ret_val = 0;

    // printf("%2d - sum = 0x%X (0x%X), avg = 0x%X, new = 0x%X\n", oldest, val_sum , val_sum >>5, average, new_val);

    /* kick-off the oldest one and insert the new one */
    val_sum -= adc_val[oldest];
    val_sum += new_val;
    adc_val[oldest] = new_val;
    oldest = (oldest + 1) % SNUM;

    return ret_val;
}

uint8_t adc_trng_gen_rnd_8(void)
{
    int       i;
    uint8_t  val8;

    val8 = 0;
    for (i = 7; i >= 0; i--)
        val8 |= (adc_trng_gen_bit() << i);

    // printf("%2d - sum = 0x%X, val8 = 0x%X\n", oldest, val_sum, val8);

    return val8;
}

uint16_t adc_trng_gen_rnd_16(void)
{
    int       i;
    uint16_t  val16;

    val16 = 0;
    for (i = 15; i >= 0; i--)
        val16 |= (adc_trng_gen_bit() << i);

    // printf("%2d - sum = 0x%X, val16 = 0x%X\n", oldest, val_sum, val16);

    return val16;
}

uint32_t adc_trng_gen_rnd_32(void)
{
    int       i;
    uint32_t  val32;

    val32 = 0;
    for (i = 31; i >= 0; i--)
        val32 |= (adc_trng_gen_bit() << i);

    // printf("%2d - sum = 0x%X, val32 = 0x%X\n", oldest, val_sum, val32);

    return val32;
}

void ADC_InitChannel(void)
{
    int i = 0;
    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /*Wait for ADC internal power ready*/
    CLK_SysTickDelay(10000);

    /* Set input mode as single-end, Single mode, and select channel 29 (band-gap voltage) */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT29);
    
    /* To sample band-gap precisely, the ADC capacitor must be charged at least 3 us for charging the ADC capacitor ( Cin )*/
    /* Sampling time = extended sampling time + 1 */
    /* 1/24000000 * (Sampling time) = 3 us */
	/*
	    printf("+----------------------------------------------------------------------+\n");
	    printf("|   ADC clock source -> PCLK1  = 48 MHz                                |\n");
	    printf("|   ADC clock divider          = 2                                     |\n");
	    printf("|   ADC clock                  = 48 MHz / 2 = 24 MHz                   |\n");
	    printf("|   ADC extended sampling time = 71                                    |\n");
	    printf("|   ADC conversion time = 17 + ADC extended sampling time = 88         |\n");
	    printf("|   ADC conversion rate = 24 MHz / 88 = 272.7 ksps                     |\n");
	    printf("+----------------------------------------------------------------------+\n");
	*/

    /* Set extend sampling time based on external resistor value.*/
    ADC_SetExtendSampleTime(ADC,(uint32_t) NULL, 71);

    val_sum = 0;
    for (i = 0; i < SNUM; i++)
    {
        adc_val[i] = get_adc_bg_val();
        // printf("int adc val = 0x%x\n", adc_val[i]);
        val_sum += adc_val[i];
    }
    oldest = 0;

    adc_trng_gen_rnd_16();    // drop the first 32-bits
}

// Fast 0-255 random number generator from http://eternityforest.com/Projects/rng.php:
uint16_t rng(void)//void uint8_t __attribute__((always_inline)) rng(void)
{
    zx++;
    za = (za^zc^zx);
    zb = (zb+za);
    zc = (zc+(zb>>1)^za);
    return zc;
}

void prepare_seed(void)
{
    za = adc_trng_gen_rnd_16(); 
    zb = adc_trng_gen_rnd_16();  
    zc = adc_trng_gen_rnd_16();  
    zx = adc_trng_gen_rnd_16();      
}

uint32_t random(int min, int max)
{
    uint32_t res= 0;    
    uint32_t length_of_range = 0;
    uint16_t adc_vaule = 0;
    uint16_t seed = 0;

    length_of_range = max - min + 1;

    adc_vaule = adc_trng_gen_rnd_16();
    seed = rng();
    srand(seed + adc_vaule);

    res = (uint32_t)(rand() % length_of_range + min);

    #if 1   // debug
    printf("adc_vaule:0x%4X,range:%5d(min:%5d,max:%5d) [0x%4X/0x%4X/0x%4X/0x%4X/0x%4X]\r\n" ,
            adc_vaule , res , min , max , 
            za , zb ,zc , zx , seed);
    #endif

    return res;
}

void loop(void)
{
    static uint16_t counter = 0;
    static uint16_t state = 0;
    uint16_t range_max = 0;
    if (is_flag_set(flag_generate_data))
    {
        set_flag(flag_generate_data,DISABLE);

        switch(state)
        {
            case 0:
                range_max = 0x0FFF;
                random(0 , range_max);
                break;
            case 1:
                range_max = 0x1FFF;
                random(0 , range_max); 
                break;
            case 2:
                range_max = 0x3FFF;
                random(0 , range_max);
                break;
            case 3:
                range_max = 0x7FFF;
                random(0 , range_max);
                break;            
        }

        if (counter++ >= 100)
        {
            counter = 0;
            state++;
            if (state == 4)
            {
                state = 0;
            }
        }        
    }
}

void GPIO_Init(void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

}

void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;

	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);

			PB14 ^= 1;
		}

		if ((get_tick() % 100) == 0)
		{
            set_flag(flag_generate_data , ENABLE);
		}	
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
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
            case '1':

                break;
			case 'X':
			case 'x':
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
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR0_MODULE);
  	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(ADC_MODULE);	
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(2));	

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

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

    GPIO_Init();
    UART0_Init();
    TIMER1_Init();
    // TIMER3_Init();

	ADC_InitChannel(); 
    prepare_seed();   

    /* Got no where to go, just loop forever */
    while(1)
    {    

        loop();        

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
