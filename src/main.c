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
void myGPIOA_Init(void);
void myTIM2_Init(void);
void myTIM16_Init(void);
void myEXTI_Init(void);

/* Global vars */
float gbl_sigFreq = 0;

int
main(int argc, char* argv[])
{
	trace_printf("Welcome to the final project.\n");
	if (VERBOSE) trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Init I/O port PA for input sig */
	myTIM2_Init();		/* Init timer for input sig period measurement */
	myEXTI_Init();		/* Init EXTI to trigger on input sig waveform edge */
	LCD_Init();         /* Init LCD communication through SPI & write placeholder */
	myTIM16_Init();     /* Init & start LCD update timer */

    while (1) {
        // Nothing is going on here...
    }

    return 0;

}

void myGPIOA_Init()
{
    /* Enable clock for GPIOA peripheral */
    // Relevant register: RCC->AHBENR
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    /* Configure PA1 as input */
    // Relevant register: GPIOA->MODER
    GPIOA->MODER &= ~(GPIO_MODER_MODER1);

    /* Ensure no pull-up/pull-down for PA1 */
    // Relevant register: GPIOA->PUPDR
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}

void myTIM2_Init()
{
    /* Enable clock for TIM2 peripheral */
    // Relevant register: RCC->APB1ENR
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure TIM2: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    // Relevant register: TIM2->CR1
    TIM2->CR1 = ((uint16_t) 0x008C);

    /* Set clock prescaler value */
    TIM2->PSC = myTIM2_PRESCALER;
    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

    /* Update timer registers */
    // Relevant register: TIM2->EGR
    TIM2->EGR = ((uint16_t) 0x0001);

    /* Assign TIM2 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority
    NVIC_SetPriority(TIM2_IRQn, 0);

    /* Enable TIM2 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable update interrupt generation */
    // Relevant register: TIM2->DIER
    TIM2->DIER |= TIM_DIER_UIE;
}

void myTIM16_Init()
{
    /* Enable clock for TIM16 peripheral */
    // Relevant register: RCC->APB2ENR
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

    /* Configure TIM16: buffer auto-reload, count up, stop on overflow,
     * enable update events, interrupt on overflow only */
    // Relevant register: TIM2->CR1
    TIM16->CR1 = ((uint16_t) 0x008C);

    /* Set clock prescaler value */
    TIM16->PSC = ONE_MS_PER_TICK_PRESCALER;
    /* Set auto-reloaded delay */
    TIM16->ARR = LCD_UPDATE_PERIOD_MS;

    /* Update timer registers */
    // Relevant register: TIM2->EGR
    TIM16->EGR = ((uint16_t) 0x0001);

    /* Assign TIM16 interrupt priority = 1 in NVIC */
    // Relevant register: NVIC->IP[3], or use NVIC_SetPriority
    NVIC_SetPriority(TIM16_IRQn, 1);

    /* Enable TIM16 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(TIM16_IRQn);

    /* Enable update interrupt generation */
    // Relevant register: TIM16->DIER
    TIM16->DIER |= TIM_DIER_UIE;

    /* Activate timer! */
    TIM16->CR1 |= TIM_CR1_CEN;
}

void myEXTI_Init()
{
    /* Map EXTI1 line to PA1 */
    // Relevant register: SYSCFG->EXTICR[0]
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

    /* EXTI1 line interrupts: set rising-edge trigger */
    // Relevant register: EXTI->RTSR
    EXTI->RTSR |= EXTI_RTSR_TR1;

    /* Unmask interrupts from EXTI1 line */
    // Relevant register: EXTI->IMR
    EXTI->IMR |= EXTI_IMR_MR1;

    /* Assign EXTI1 interrupt priority = 0 in NVIC */
    // Relevant register: NVIC->IP[1], or use NVIC_SetPriority
    NVIC_SetPriority(EXTI0_1_IRQn, 0);

    /* Enable EXTI1 interrupts in NVIC */
    // Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
    /* Check if update interrupt flag is indeed set */
    if ((TIM2->SR & TIM_SR_UIF) != 0) {
        trace_printf("\n*** Overflow! ***\n");

        /* Clear update interrupt flag */
        // Relevant register: TIM2->SR
        TIM2->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        // Relevant register: TIM2->CR1
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

void TIM16_IRQHandler()
{
    /* Check if update interrupt flag is indeed set */
    if ((TIM16->SR & TIM_SR_UIF) != 0) {
        if (VERBOSE) trace_printf("\nUpdating LCD with freq: %f Hz\n", gbl_sigFreq);
        LCD_UpdateFreq(gbl_sigFreq);

        /* Clear update interrupt flag */
        // Relevant register: TIM16->SR
        TIM16->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        // Relevant register: TIM16->CR1
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

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
