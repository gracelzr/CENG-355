//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include "stdio.h"
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_conf.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

#define REFRESH_RATE 500000
//void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);//Initial ADC
void myDAC_Init(void);// Initialize DAC
void ADCreading(void);//ADC reading
void DACwriting(void);//DAC reading
void mySPI_StructInit(void);//Initialize SPI
void LCD_Init(void);//Initialize LCD
void LCD_Com(uint8_t variable);//Send LCD Command Instruction
void LCD_Comtrans(uint8_t variable);//Convert LCD Command to recognizable forms
void LCD_Trans(uint32_t variable);//Convert LCD data to recognizable forms
void Delay_tool(unsigned int i);
void LCD_Display();


//Type here

// Your global variables...
	double pulseperiod;
	double pulsefrequency;
	unsigned int pulseflag = 1;
	unsigned int ADC_reading = 0;
	unsigned int R_value = 0;
	unsigned int LCD_delay = 80000;
	uint8_t HexNum[8];
	uint8_t H_bit;
	uint8_t L_bit;
	int edgeHit = 0;	// Keeps track of number of rising edges hit
	double frequency;	// Frequency of signal generator (Hz)
	char digits[4] = {'0', '0', '0', '0'};
//Need more value for LCD

int main(int argc, char* argv[])
{

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myADC_Init();   //Initialize ADC control
	myDAC_Init();		//Initialize DAC control
	mySPI_StructInit();	// Initialize SPI
	LCD_Init();		// Initialize LCD

	ADC1->CR |= ADC_CR_ADSTART;//Start Conversion
	while (1)
	{
		ADCreading(); //Reading from ADC port
		DACwriting();	//Writing into DAC port
		LCD_Display(); // Display data on LCD
	}

	return 0;

}

void Delay_tool(unsigned int k)
{
	while(k>0)
	{
		k--;
	}
}

void myADC_Init()
{
	RCC->APB2ENR |= RCC_APB2Periph_ADC1; // enable ADC peripheral clock

	// ADC GPIO Pins

	// Enable clock for GPIOC peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC1 as analog mode
	GPIOC->MODER |= GPIO_MODER_MODER1;

	// Ensure no pull-up/pull-down for PC1
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	ADC1->CR |= ADC_CR_ADEN; // enable ADC
	while((ADC1->ISR & ADC_ISR_ADRDY) == 1); // wait until ADC ready
	ADC1->CR &= ~(ADC_CR_ADDIS); // make sure ADC doesn't get disabled
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11; // initialize channel 11 (for GPIO PC1) to read analog signal
	ADC1->CFGR1 |= ADC_CFGR1_CONT; // Set to continuous conversion mode (constantly take samples)
	ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN); // disable hardware triggers
	ADC1->CFGR1 &= ~(ADC_CFGR1_ALIGN); // right aligned converted results in data register
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; // turn overrun mode on (enables overwriting of ADC1->DR)
	ADC1->CFGR1 |= ADC_CFGR1_RES_1; // set 8-bit resolution
	ADC1->SMPR |= ADC_SMPR_SMP; // set sampling time to 239.5 Cycles

	/*RCC->APB2ENR |= RCC_APB2ENR_ADCEN;//Enable interface clock
	ADC1->CR |= ADC_CR_ADEN; //Enable ADC command
	ADC1->CFGR1 |= ADC_CFGR1_CONT; //Set ADC as continuous conversion mode
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1;//ADC is on channel 1
	ADC1->SMPR |= 0X0002; // Set ADC clock cycle as 13.5 */

}


void myDAC_Init()
{
	RCC->APB1ENR |= RCC_APB1Periph_DAC; // enable DAC peripheral clock

	// DAC GPIO Inits

	// Enable clock for GPIOA peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Set GPIO PA4 to analog mode
	GPIOA->MODER |= GPIO_MODER_MODER4;

	// Ensure no pull-up/pull-down for PA4
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	DAC->CR |= DAC_CR_EN1; // enable DAC
	DAC->CR &= ~(DAC_CR_TEN1); // disable external trigger
	DAC->CR &= ~(DAC_CR_BOFF1); // enable output buffer

}


void ADCreading()
{
	ADC1->CR |= ADC_CR_ADSTART; //start ADC
	while((ADC1->ISR & ADC_ISR_EOC) == 0); // wait until conversion is finished
	ADC_reading = (unsigned int)(ADC1->DR);
	trace_printf("ADC Result: %u \n", ADC_reading); // for testing

	R_value = ADC_reading*5000/255;

}


void DACwriting()
{
	//DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1; //Enable DAC trigger
	trace_printf("Resistor Result: %u \n", R_value); // for testing
	DAC->DHR8R1 = ADC_reading; //ADC to DAC right-aligned

}

void mySPI_StructInit()
{
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	// SPI GPIO pins
	// Enable clock for GPIOB peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Configure PB5 as alternate function for MOSI
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL5);
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	// Ensure no pull-up/pull-down for PB5
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);

	// Configure PB3 as alternate function for SCK
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3);
	GPIOB->MODER |= GPIO_MODER_MODER3_1;

	// Ensure no pull-up/pull-down for PB3
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);

	// Configure PB4 as output for LCK
	GPIOB->MODER |= GPIO_MODER_MODER4_0;

	// Ensure no pull-up/pull-down for PB4
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	// Initialize SPI
	// SPI clock enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI_StructInit(SPI_InitStruct); // initializes all SPI_InitStruct members to default values
	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);

}

void LCD_Update(char data)
{
	GPIOB->BRR = GPIO_BRR_BR_4; // LCK = 0
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET); // Wait until SPI1 is ready
	SPI_SendData8(SPI1, data); // transmits data
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)==SET); // Wait until not busy
	GPIOB->BSRR = GPIO_BSRR_BS_4; // LCK = 1
}

void LCD_Trans(uint32_t num) {

	int i = 0; // the array index
	digits[0] = '0';
	digits[1] = '0';
	digits[2] = '0';
	digits[3] = '0';
	// loop till there's nothing left
	while (num) {
    		digits[i++] = (num % 10) + '0'; // assign the last digit
			num /= 10; // "right shift" the number by one digit
	}
}

void LCD_Com(uint8_t variable){
// split instructions into two 4 bit halves
	uint8_t low_nibble, high_nibble;
	high_nibble = (variable & 0xF0) >> 4; // upper half
	low_nibble = (variable & 0x0F); // lower half

	// top half send to SPI
	LCD_Update(high_nibble); // RS = 0 and EN = 0
	Delay_tool(100000);
	LCD_Update(high_nibble | 0x80); // RS = 0 and EN = 1
	Delay_tool(100000);
	LCD_Update(high_nibble); // RS = 0 and EN = 0
	Delay_tool(100000);
	// bottom half send to SPI
	LCD_Update(low_nibble); // RS = 0 and EN = 0
	Delay_tool(100000);
	LCD_Update(low_nibble | 0x80); // RS = 0 and EN = 1
	Delay_tool(100000);
	LCD_Update(low_nibble); // RS = 0 and EN = 0
	Delay_tool(100000);
}

void LCD_Comtrans(uint8_t variable) {

	// split ascii character into two 4 bit halves
	uint8_t low_nibble, high_nibble;

	high_nibble = (variable & 0xF0) >> 4; // upper half
	low_nibble = (variable & 0x0F); // lower half
	// top half send to SPI
	LCD_Update(high_nibble | 0x40); // RS = 1 and EN = 0
	Delay_tool(30000);
	LCD_Update(high_nibble | 0xC0); // RS = 1 and EN = 1
	Delay_tool(30000);
	LCD_Update(high_nibble | 0x40); // RS = 1 and EN = 0
	Delay_tool(30000);

	// bottom half send to SPI
	LCD_Update(low_nibble | 0x40); // RS = 1 and EN = 0
	Delay_tool(30000);
	LCD_Update(low_nibble | 0xC0); // RS = 1 and EN = 1
	Delay_tool(30000);
	LCD_Update(low_nibble | 0x40); // RS = 1 and EN = 0
	Delay_tool(30000);
}


void LCD_Init(){
	LCD_Update(0x2); // Let EN = 0, RS = 0, DB[7:4] = 0010
	LCD_Update(0x82); // Let EN = 1, RS = 0, DB[7:4] = 0010
	LCD_Update(0x2); // Let EN = 0, RS = 0, DB[7:4] = 0010

	Delay_tool(30000); // ~10 ms

	// DDRAM access is performed using 4-bit interface
	// and two lines of eight characters are displayed
	LCD_Com(0x28);

	// display is on, cursor is displayed, and it is not blinking
	LCD_Com(0x0E);

	// DDRAM address is auto-incremented after each access
	// and it is not shifted
	LCD_Com(0x06);

	// Clear display
	LCD_Com(0x01);
	Delay_tool(100000);
}


void LCD_Display() {

	int i; // iterator
	LCD_Com((char)0x80); // set data to first line of LCD
	LCD_Comtrans('F');
	LCD_Comtrans(':');
	LCD_Trans(frequency); // convert frequency to ASCII digits array
	for(i = 3; i >= 0; i--) {
		LCD_Comtrans(digits[i]); // Writing frequency to LCD
	}
	LCD_Comtrans('H');
	LCD_Comtrans('z');

	LCD_Com((char)0xC0); // set data to second line of LCD
	LCD_Comtrans('R');
	LCD_Comtrans(':');
	LCD_Trans(R_value); // convert resistance to ASCII digits array
	for(i = 3; i >= 0; i--) {
		LCD_Comtrans(digits[i]); // Writing resistance to LCD
	}
	LCD_Comtrans('o');
	LCD_Comtrans('h');

	Delay_tool(REFRESH_RATE); // refresh rate ~250 ms
}

// GPIOA Section
// Enable clock for GPIOA peripheral
void myGPIOA_Init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA1 as input
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	// Ensure no pull-up/pull-down for PA1
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}

// Enable clock for TIM2 peripheral
// TIM2 Section
// Configure TIM2: buffer auto-reload, count up, stop on overflow,
// enable update events, interrupt on overflow only
void myTIM2_Init()
{
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	// Set clock prescaler value
	TIM2->PSC = myTIM2_PRESCALER;
	// Set auto-reloaded delay
	TIM2->ARR = myTIM2_PERIOD;

	// Update timer registers
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	// Assign TIM2 interrupt priority = 0 in NVIC
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	// Enable TIM2 interrupts in NVIC
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	// Enable update interrupt generation
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}

// EXTI Section
// Map EXTI1 line to PA1
// Relevant register: SYSCFG->EXTICR[0]
//copy from part 2
void myEXTI_Init()
{
	SYSCFG->EXTICR[0] &= ~(0xF0);

	// EXTI1 line interrupts: set rising-edge trigger
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= 0x2;

	// Unmask interrupts from EXTI1 line
	// Relevant register: EXTI->IMR
	EXTI->IMR |= 0x2;

	// Assign EXTI1 interrupt priority = 0 in NVIC
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn , 0);

	// Enable EXTI1 interrupts in NVIC
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

}

void TIM2_IRQHandler()
{
	// Check if update interrupt flag is indeed set
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		// Clear update interrupt flag
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		// Restart stopped timer
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

// This handler is declared in system/src/cmsis/vectors_stm32f0xx.c
void EXTI0_1_IRQHandler()
{
	// Your local variables...
	double count;	// Keep track of ticks elapsed between edges
	double period;	// Period of signal generator (s)
	int microPeriod;	// Period of signal generator (us)
	int milliFreq;	// Frequency of signal generator (mHz)

	// Check if EXTI1 interrupt pending flag is indeed set
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		// A rising edge has occurred
		edgeHit++;

		// 1. If this is the first edge:
		if (edgeHit == 1) {
			//	- Clear count register (TIM2->CNT).
			TIM2->CNT = 0x0;
			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;

		}

		//    Else (this is the second edge):
		else {
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			//	- Read out count register (TIM2->CNT).
			count = (TIM2->CNT);
			//	- Calculate signal period and frequency.
			period = count/((double)SystemCoreClock);
			frequency = 1.0/period;
			//	- Convert period to us and frequency to mHz
			microPeriod = (int)(period*1000000.0);
			milliFreq = (int)(frequency*1000);
			//	- Print calculated values to the console.
			//	  NOTE: Function trace_printf does not work
			//	  with floating-point numbers: you must use
			//	  "unsigned int" type to print your signal
			//	  period and frequency.
			//trace_printf("Signal Generator Period: %u us\n", ((unsigned int)microPeriod));
			//trace_printf("Signal Generator Frequency: %u mHz\n", (unsigned int)milliFreq);
			//
			edgeHit = 0;
		}
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

