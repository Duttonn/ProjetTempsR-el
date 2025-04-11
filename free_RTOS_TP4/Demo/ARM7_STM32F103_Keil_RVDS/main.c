#include <stdlib.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "def_type_gpio.h"
#include "myTasks.h"

#define GPIO_MODE_INPUT_PULLUP 0x8

#define mainLED_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define NUM_LEDS 60
#define BITS_PER_LED 24
#define TOTAL_BITS (NUM_LEDS * BITS_PER_LED)

// Tableau compact 1 bit par LED (8 octets suffisent pour 60 LEDs)
uint8_t ledStatus[60] = {0};

// Buffer PWM pour envoyer les bits aux WS2812B
uint8_t pwmBuffer[TOTAL_BITS];
volatile uint16_t currentBit = 0;
volatile uint8_t sending = 0;

void init_Timer2_PWM(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Activer l'horloge du Timer2
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Activer GPIOA

    GPIOA->CRL &= ~(0xF); // Effacer configuration PA0
    GPIOA->CRL |= (0xB);  // Mode Output AF Push-Pull 50MHz

    TIM2->PSC = 0;   // Pas de pr�division (72MHz direct)
    TIM2->ARR = 100; // Augmente la p�riode pour atteindre ~1.4 �s  152 OLDVALUE
    TIM2->CCR1 = 30; // PWM � 30% par d�faut (envoyer "0")

    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Mode PWM 1
    TIM2->CCER |= TIM_CCER_CC1E;                        // Activer sortie sur PA0

    TIM2->DIER |= TIM_DIER_UIE; // Activer IT update
    NVIC_EnableIRQ(TIM2_IRQn);  // Activer interruption NVIC

    TIM2->CR1 |= TIM_CR1_CEN; // D�marrer le timer
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF; // Acquittement

        if (currentBit < TOTAL_BITS)
        {
					TIM2->CCR1 = pwmBuffer[currentBit] ? 70:30;//<(0.7 * TIM2->ARR) : (0.3 * TIM2->ARR);
            currentBit++;
        }
        else
        {
            TIM2->CCR1 = 0; // RESET des LEDs (40 cycles bas)
            if (++currentBit > TOTAL_BITS + 40)
            {
                currentBit = 0;
                sending = 0;
                TIM2->CR1 &= ~TIM_CR1_CEN; // Désactiver le Timer
            }
        }
    }
}

void update_ledBuffer(void)
{
    int i, j;
    for (i = 0; i < NUM_LEDS; i++)
    {
        
        uint8_t ledValue = ledStatus[i];

        for (j = 0; j < BITS_PER_LED; j++)
        {
            pwmBuffer[i * BITS_PER_LED + j] = (ledValue) ? 1 : 0;
        }
    }
}

void stop_ledTransmission(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN; // Désactiver le Timer
    sending = 0;
}

void start_ledTransmission(void)
{
    if (!sending)
    {
        sending = 1;
        currentBit = 0;
        TIM2->CR1 |= TIM_CR1_CEN; // <<< ⚠️ Cette ligne doit être présente !
    }
}

void init_gpio(void)
{	
	//init_GPIOx(GPIOB, 0, GPIO_MODE_INPUT_PULLUP); 
    init_GPIOx(GPIOB, 0, GPIO_MODE_OUTPUT_PP_50MHz);

    //init_GPIOx(GPIOA, 2, GPIO_MODE_INPUT_PULLUP); 
	
	
    init_GPIOx(GPIOA, 1, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOA, 2, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOA, 4, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOA, 5, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOA, 6, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOA, 7, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOB, 10, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOB, 11, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOB, 12, GPIO_MODE_OUTPUT_PP_50MHz);
    init_GPIOx(GPIOB, 13, GPIO_MODE_OUTPUT_PP_50MHz);
}

static void prvSetupHardware(void)
{
    init_gpio();
    init_Timer2_PWM();
}

int main(void)
{
    static uint32_t oscille_plantage = 1 << 13;
    prvSetupHardware();

    xTaskCreate(taskScrutBoutons, "ScrutBoutons", 128, NULL, 4, NULL);
    xTaskCreate(taskGestionCasiers, "GestionCasiers", 256, NULL, 2, &xTaskGestionCasiersHandle);
    xTaskCreate(taskMiseAJourLEDs, "MiseAJourLEDs", 128, NULL, 2, NULL);
    xTaskCreate(taskGestionReedSwitches, "GestionReed", 128, NULL, 2, NULL);
    xTaskCreate(taskSimulateButtons, "SimButtons", 128, NULL, 3, NULL);

    // Mise � jour du buffer et d�marrage de la transmission
    // update_ledBuffer();
    // start_ledTransmission();

    vInit_myTasks(mainLED_TASK_PRIORITY);
    vTaskStartScheduler();

    for (;;)
    {
        GPIOB->BSRR = oscille_plantage;
        oscille_plantage ^= 0x00010001 << 13;
    }
}
