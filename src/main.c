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

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "lcd.h"

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


// ----------------------------------------------------------------------------
//                      DEFINES
// ----------------------------------------------------------------------------

#ifndef VERBOSE
#define VERBOSE 0
#endif

/* Clock prescalers */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
#define ONE_MS_PER_TICK_PRESCALER ((uint16_t)((SystemCoreClock - 1) / 1000))
/* Clock periods */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF) // max value before overflow
#if VERBOSE
#define LCD_UPDATE_PERIOD_MS ((uint32_t)2500)
#else
#define LCD_UPDATE_PERIOD_MS ((uint32_t)250)
#endif
#define RESISTANCE_MAX_VALUE (5000.0) // PBMCUSLK uses a 5k pot
#define ADC_MAX_VALUE ((float)(0xFFF)) // ADC bit resolution (12 by default)
#define DAC_MAX_VALUE ((float)(0xFFF)) // DAC bit resolution (12 by default)
#define DAC_MAX_VOLTAGE (2.95) // measured output from PA4 when DAC->DHR12R1 = DAC_MAX_VALUE
#define OPTO_DEADBAND_END_VOLTAGE (1.0) // voltage needed to overcome diode drops in opto

// ----------------------------------------------------------------------------
//                      PROTOTYPES
// ----------------------------------------------------------------------------

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myTIM16_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
uint32_t getPotADCValue(void);
uint32_t applyOptoOffsetToDAC(uint32_t rawDAC);

// ----------------------------------------------------------------------------
//                      GLOBAL VARIABLES
// ----------------------------------------------------------------------------

float gbl_sigFreq = 0.0;
float gbl_resistance = 0.0;

int
main(int argc, char* argv[])
{
	trace_printf("Welcome to the final project.\n");
	if (VERBOSE) trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Init I/O port PA for input sig, ADC, DAC */
	myADC_Init();       /* Init ADC for continuous measurement */
	myDAC_Init();       /* Init DAC to output timer control voltage */
	myTIM2_Init();		/* Init timer for input sig period measurement */
	myEXTI_Init();		/* Init EXTI to trigger on input sig waveform edge */
	LCD_Init();         /* Init LCD communication through SPI & write placeholder */
	myTIM16_Init();     /* Init & start LCD update timer */

    while (1) {
        uint32_t potADCValue = getPotADCValue();

        // Use the ADC value for DAC but offset it to avoid output voltages that lie
        // in the deadband for the optocoupler (around 0->1V). This will allow all
        // values of the pot to correspond to a change in timer freq.
        uint32_t timerControlDACValue = applyOptoOffsetToDAC(potADCValue);

        // Update the DAC value
        // DHR12R1: "Data Holding Register, 12b, Right aligned, Channel 1"
        DAC->DHR12R1 = timerControlDACValue;

        // Convert to resistance range
        float normalizedPotADC = (((float)potADCValue) / ADC_MAX_VALUE);
        gbl_resistance = normalizedPotADC * RESISTANCE_MAX_VALUE;

        if (VERBOSE) trace_printf("ADC Value: %d\n", potADCValue);
        if (VERBOSE) trace_printf("Resistance: %f\n", gbl_resistance);
    }

    return 0;

}

void myGPIOA_Init()
{
    // Configure:
    //   PA0 --ana-> Analog in to ADC for resistance
    //   PA1 --in--> Read 555 timer edge transitions
    //   PA4 --ana-> Analog out from DAC for timer control voltage

    /* Enable clock for GPIOA peripheral */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Configure PA0 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);  /* Analog input */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);  /* No pull up/down */

    /* Configure PA1 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER1);  /* Input */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);  /* No pull up/down */

    /* Configure PA4 */
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);  /* Analog output */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);  /* No pull up/down */
}

void myTIM2_Init()
{
    /* Enable clock for TIM2 peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    TIM2->CR1 = ((uint16_t) 0x008C);

    /* Set clock prescaler value */
    TIM2->PSC = myTIM2_PRESCALER;
    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

    /* Update timer registers */
    TIM2->EGR = ((uint16_t) 0x0001);

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable TIM2 interrupts in NVIC */
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    TIM2->DIER |= TIM_DIER_UIE;
}

void myTIM16_Init()
{
    /* Enable clock for TIM16 peripheral */
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

    /* Configure TIM16: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    TIM16->CR1 = ((uint16_t) 0x008C);

    /* Set clock prescaler value */
    TIM16->PSC = ONE_MS_PER_TICK_PRESCALER;
    /* Set auto-reloaded delay */
    TIM16->ARR = LCD_UPDATE_PERIOD_MS;

    /* Update timer registers */
    TIM16->EGR = ((uint16_t) 0x0001);

    /* Assign TIM16 interrupt priority = 1 in NVIC */
    /* Will be interrupted by P0 tasks, like edge measurements */
    NVIC_SetPriority(TIM16_IRQn, 1);

    /* Enable TIM16 interrupts in NVIC */
    NVIC_EnableIRQ(TIM16_IRQn);

    /* Enable update interrupt generation */
    TIM16->DIER |= TIM_DIER_UIE;

    /* Activate timer! */
    TIM16->CR1 |= TIM_CR1_CEN;
}

void myEXTI_Init()
{
    /* Map EXTI1 line to PA1 */
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

    /* EXTI1 line interrupts: set rising-edge trigger */
    EXTI->RTSR |= EXTI_RTSR_TR1;

    /* Unmask interrupts from EXTI1 line */
    EXTI->IMR |= EXTI_IMR_MR1;

    /* Assign EXTI1 interrupt priority = 0 in NVIC */
    NVIC_SetPriority(EXTI0_1_IRQn, 0);

    /* Enable EXTI1 interrupts in NVIC */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void myADC_Init(){
    /* Enable clock for ADC */
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    /* Tell ADC to begin self-calibration and wait for it to finish */
    if (VERBOSE) trace_printf("Start ADC calibration...\n");
    ADC1->CR = ADC_CR_ADCAL;
    while (ADC1->CR == ADC_CR_ADCAL) {};
    if (VERBOSE) trace_printf("ADC calibration finished!\n\n");

    /* ADC Configuration:
     *   - Continuous conversion
     *   - Overrun mode
     */
    ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

    /* Select channel, PA0 needs Channel 0 */
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;

    /* Enable ADC and wait for it to have ready status */
    if (VERBOSE) trace_printf("Start ADC enable...\n");
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {};
    if (VERBOSE) trace_printf("ADC enable finished!\n\n");
}

void myDAC_Init() {
    /* Enable clock for DAC */
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    /* Enable DAC */
    DAC->CR |= DAC_CR_EN1;
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
    /* Check if update interrupt flag is indeed set */
    if ((TIM2->SR & TIM_SR_UIF) != 0) {
        trace_printf("\n*** 555 Timer Input Period Overflow ***\n");

        /* Clear update interrupt flag */
        TIM2->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

void TIM16_IRQHandler()
{
    /* Check if update interrupt flag is indeed set */
    if ((TIM16->SR & TIM_SR_UIF) != 0) {
        if (VERBOSE) trace_printf("\nUpdating LCD with freq: %f Hz\n", gbl_sigFreq);
        LCD_UpdateFreq(gbl_sigFreq);

        if (VERBOSE) trace_printf("Updating LCD with resistance: $f Ohm\n\n");
        LCD_UpdateResistance(gbl_resistance);

        /* Clear update interrupt flag */
        TIM16->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        TIM16->CR1 |= TIM_CR1_CEN;
    }
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		// timer will be disabled for first edge and enabled on second
		uint16_t isTimerEnabled = (TIM2->CR1 & TIM_CR1_CEN);

		if (isTimerEnabled) {
			// stop timer and get count
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			uint32_t count = TIM2->CNT;
			gbl_sigFreq = ((float)SystemCoreClock) / count;
			float sigPeriod = 1.0 / gbl_sigFreq;

			if (VERBOSE)trace_printf("Signal Freq:   %f Hz\n", gbl_sigFreq);
			if (VERBOSE)trace_printf("Signal Period: %f s\n\n", sigPeriod);

		} else {
			// reset & start timer
			TIM2->CNT = (uint32_t)0x0;
			TIM2->CR1 |= TIM_CR1_CEN;
		}

		// clear EXTI interrupt pending flag
		EXTI->PR |= EXTI_PR_PR1;
	}
}

uint32_t getPotADCValue(){
    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for End Of Conversion flag to be set
    while (!(ADC1->ISR & ADC_ISR_EOC)) {
        /* loop until conversion is complete */
    };

    // Reset End Of Conversion flag
    ADC1->ISR &= ~(ADC_ISR_EOC);

    // Apply the data mask to the data register
    return ((ADC1->DR) & ADC_DR_DATA);
}

uint32_t applyOptoOffsetToDAC(uint32_t rawDAC){
    // map the DAC value -> voltage range (opto deadband to max output)
    float normalizedDAC = rawDAC / DAC_MAX_VALUE;
    float outputVoltageRange = DAC_MAX_VOLTAGE - OPTO_DEADBAND_END_VOLTAGE;
    float outputVoltage = (normalizedDAC * outputVoltageRange) + OPTO_DEADBAND_END_VOLTAGE;

    // TODO: Add some assertions for the output voltage
    //   1. voltage >= 0
    //   2. voltage >= deadband limit
    //   3. voltage <= max output voltage

    if (VERBOSE) trace_printf("\nOutput voltage: %f", outputVoltage);

    // convert the voltage value back to a DAC level
    float normalizedOutputVoltage = outputVoltage / DAC_MAX_VOLTAGE;
    float outputDACValue = normalizedOutputVoltage * DAC_MAX_VALUE;

    return ((uint32_t)outputDACValue);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
