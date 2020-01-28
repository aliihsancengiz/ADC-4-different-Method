#include "stm32f4xx_hal.h"  
#include<stdio.h>
#include<string.h>

//Handlers for peripheral
RCC_OscInitTypeDef hOsc;
RCC_ClkInitTypeDef hClk;
GPIO_InitTypeDef hgpio;
UART_HandleTypeDef huart;
ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef hcadc;

//Configureation functions
static void connectHSI2PLL(void);
static void configureSysClk(void);
static void configureADC(void);
static void configureUART(void);
void ErrorHandler(void);

//For storing and traansmitting adc data
uint32_t rawdata;
char msg[20];
int adccompleted=0;

int main()
{
	//Initialize the peripheral
	HAL_Init();
	
	//Set APB busses to 21MHz
	connectHSI2PLL();
	configureSysClk();
	SystemCoreClockUpdate();
	
	//Enabling systick interrupt beecause of many periphreal use it for timeout
	HAL_NVIC_EnableIRQ(SysTick_IRQn);
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
	//Enable ADC IRQ
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_NVIC_SetPriority(ADC_IRQn,0,0);
	
	//Configure peripheral
	configureADC();
	configureUART();
	
	//Start adc with interrput mode at the end of each conversion adc will generate an interrupt
	HAL_ADC_Start_IT(&hadc);
	while(1)
	{
		//Check for completion of adc	
		if(adccompleted)
		{
			rawdata=HAL_ADC_GetValue(&hadc);
			sprintf(msg, "rawValue: %hu\r\n", rawdata);
			//Send through uart
			HAL_UART_Transmit(&huart, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
			adccompleted=0;
		}

		
	}
		
		
		

}

void configureADC(void)
{
	//Enable ADC and GPIOA clock
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	//Configure PA0 as Analog mode
	hgpio.Mode=GPIO_MODE_ANALOG;
	hgpio.Pin=GPIO_PIN_0;
	hgpio.Pull=GPIO_NOPULL;
	hgpio.Speed=GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA,&hgpio);
	
	//Set adc mode for 1 continious conversion
	hadc.Instance=ADC1;
	hadc.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.ContinuousConvMode=ENABLE;
	hadc.Init.ScanConvMode=DISABLE;
	hadc.Init.DataAlign=ADC_DATAALIGN_RIGHT;
	hadc.Init.DiscontinuousConvMode=DISABLE;
	hadc.Init.NbrOfConversion=1;
	hadc.Init.DMAContinuousRequests=DISABLE;
	hadc.Init.Resolution=ADC_RESOLUTION_12B;
	hadc.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc);
	
	//Configure channel property
	hcadc.Channel=ADC_CHANNEL_0;
	hcadc.Rank=1;
	hcadc.SamplingTime=ADC_SAMPLETIME_84CYCLES;
	HAL_ADC_ConfigChannel(&hadc,&hcadc);
	
}
static void configureUART(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	
	//Gpio confiiguration for usart
	hgpio.Mode=GPIO_MODE_AF_PP;
	hgpio.Pin=GPIO_PIN_2 | GPIO_PIN_3;
	hgpio.Alternate=GPIO_AF7_USART2;
	hgpio.Speed=GPIO_SPEED_FAST;
	hgpio.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA,&hgpio);
	
	//115200-no parity 8+1 rx tx mode
	huart.Instance=USART2;
	huart.Init.BaudRate=115200;
	huart.Init.Mode=UART_MODE_TX_RX;
	huart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
	huart.Init.OverSampling=UART_OVERSAMPLING_16;
	huart.Init.Parity=UART_PARITY_NONE;
	huart.Init.StopBits=UART_STOPBITS_1;
	huart.Init.WordLength=UART_WORDLENGTH_8B;
	HAL_UART_Init(&huart);
}
static void connectHSI2PLL(void)
{
	//connect HSI to the PLL
	hOsc.OscillatorType=RCC_OSCILLATORTYPE_HSI;
	hOsc.HSIState=RCC_HSI_ON;
	hOsc.HSICalibrationValue=16;
	hOsc.PLL.PLLState=RCC_PLL_ON;
	hOsc.PLL.PLLSource=RCC_PLLSOURCE_HSI;
	hOsc.PLL.PLLM=16;
	hOsc.PLL.PLLN=336;
	hOsc.PLL.PLLP=2;
	HAL_RCC_OscConfig(&hOsc);
	return;
}
static void configureSysClk(void)
{
	//Sellct Hclock source as PLL Clock and set it 168M Hz
	hClk.ClockType=RCC_CLOCKTYPE_SYSCLK;
	hClk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
	hClk.APB1CLKDivider=8;	//Set apb1 clock to 21Mhz
	hClk.APB2CLKDivider=8;	//Set apb2 clock to 21Mhz
	if(HAL_RCC_ClockConfig(&hClk,FLASH_LATENCY_5)==HAL_ERROR)
	{
		ErrorHandler();
	}
	RCC->CFGR |=(0x1800 | 0xC000);
	return;
}
void SysTick_Handler()
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
void ADC_IRQHandler(void)
{
		HAL_ADC_IRQHandler(&hadc);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adccomplete=1;
}
void ErrorHandler(void)
{
		while(1);
	
}
