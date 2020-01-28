#include "stm32f4xx_hal.h"  
#include<stdio.h>
#include<string.h>

//Handlers for Peripheral
RCC_OscInitTypeDef hOsc;
RCC_ClkInitTypeDef hClk;
GPIO_InitTypeDef hgpio;
ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef hcadc;
TIM_HandleTypeDef htim2;
TIM_ClockConfigTypeDef hClktim2;
TIM_MasterConfigTypeDef hMstrtim2;

//Initialization Function
static void connectHSI2PLL(void);
static void configureSysClk(void);
static void configureADC(void);
static void configureTimer(void);
void ErrorHandler(void);

//Variable for storing raw data of ADC Conversion
uint16_t rawdata;

int main()
{
	//Initialize the peripheral
	HAL_Init();
	
	//Set APB busses to 21MHz
	connectHSI2PLL();
	configureSysClk();
	SystemCoreClockUpdate();
	
	//Enabling systick interrupt beecause of many periphreal use it for timeout
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	HAL_NVIC_EnableIRQ(SysTick_IRQn);
	
	configureADC();
	configureTimer();
	
	//Start the timer and conversion
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc);
	
	while(1)
	{
		
	}
}
static void  configureTimer(void)
{
	//Configuring Timer to generate 500 Hz update event this will trigger adc to make a conversion 
	//F_UE=TIM_CLK/((Prescaler+1)*Period+1)
	//F_UF=21000000/(21000*2)=500 Hz update event
	__HAL_RCC_TIM2_CLK_ENABLE();
	htim2.Instance=TIM2;
	htim2.Init.Period=1;
	htim2.Init.Prescaler=20999;
	HAL_TIM_Base_Init(&htim2);
	
	//Sellect timer clock source as internal source
	hClktim2.ClockSource=TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2,&hClktim2);
	
	//Enable Trigger Output of timer 2
	hMstrtim2.MasterOutputTrigger=TIM_TRGO_UPDATE;
	hMstrtim2.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2,&hMstrtim2);	
}
static void configureADC(void)
{
	//Enable Clock of ADC and GPIO
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	//Configure PA0 as Analog mode
	hgpio.Mode=GPIO_MODE_ANALOG;
	hgpio.Pin=GPIO_PIN_0;
	hgpio.Pull=GPIO_NOPULL;
	hgpio.Speed=GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA,&hgpio);
	
	//Configure ADC for TRGO_TIM2
	hadc.Instance=ADC1;
	hadc.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.ContinuousConvMode=DISABLE;
	hadc.Init.ScanConvMode=DISABLE;
	hadc.Init.DataAlign=ADC_DATAALIGN_RIGHT;
	hadc.Init.DiscontinuousConvMode=DISABLE;
	hadc.Init.NbrOfConversion=1;
	hadc.Init.DMAContinuousRequests=DISABLE;
	hadc.Init.Resolution=ADC_RESOLUTION_12B;
	hadc.Init.EOCSelection=ADC_EOC_SINGLE_CONV;
	hadc.Init.ExternalTrigConv=ADC_EXTERNALTRIG2_T2_TRGO;
	hadc.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;
	HAL_ADC_Init(&hadc);
	
	
	//Channel configuration
	hcadc.Channel=ADC_CHANNEL_0;
	hcadc.Rank=1;
	hcadc.SamplingTime=ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&hadc,&hcadc);
	
	//Enable ADC interrupt
	HAL_NVIC_SetPriority(ADC_IRQn,0,0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
}

static void connectHSI2PLL(void)
{
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
void ADC_IRQHandler()
{
	HAL_ADC_IRQHandler(&hadc);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	rawdata=(uint16_t)HAL_ADC_GetValue(hadc);
}
void ErrorHandler(void)
{
		while(1);
}
