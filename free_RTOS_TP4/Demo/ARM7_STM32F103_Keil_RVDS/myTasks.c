#include <stdlib.h>
#include <stdio.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>    // Pour memset
#include "stm32f10x.h" // Utilisation de CMSIS
/* le fichier .h du TP */
#include "myTasks.h"
#include "def_type_gpio.h"

volatile ModeSysteme modeActuel = MODE_EFFACE;
unsigned char etat_reed[60] = {0};
volatile int num_case_resistance = 0;
TaskHandle_t xTaskGestionCasiersHandle = NULL; // Déclaration globale

void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet)
{
    // Vérifier si le numéro de bit est dans la plage valide (0-15)
    if (num_bit > 15)
    {
        return;
    } // Gestion d'erreur, le numéro de bit doit être entre 0 et 15

    RCC->APB2ENR |= (7 << 2); // pour  GPIOA GPIOB GPIOC
    // Déterminer si le bit est dans CRL ou CRH
    if (num_bit < 8)
    {
        // CRL pour les bits 0 à 7
        uint32_t mask = 0xF << (num_bit * 4);                // Masque pour effacer les 4 bits correspondants
        uint32_t value = (uint32_t)quartet << (num_bit * 4); // Valeur à définir
        GPIOx->CRL = (GPIOx->CRL & ~mask) | value;
    }
    else
    {
        // CRH pour les bits 8 à 15
        uint32_t mask = 0xF << ((num_bit - 8) * 4);                // Masque pour effacer les 4 bits correspondants
        uint32_t value = (uint32_t)quartet << ((num_bit - 8) * 4); // Valeur à définir
        GPIOx->CRH = (GPIOx->CRH & ~mask) | value;
    }
}

void allumer_led_casier(int casier, uint32_t couleur)
{
    // Implémentation du contrôle des LEDs WS2812B
    // Ici, il faudra modifier le buffer PWM correspondant au casier
}

void taskScrutBoutons(void *pvParameters)
{
    static uint8_t last_state_test = 0;
    static uint8_t last_state_appro = 0;

    while (1)
    {
        uint8_t state_test = (GPIOA->IDR & (1 << 2)) ? 1 : 0;
        uint8_t state_appro = (GPIOA->IDR & (1 << 1)) ? 1 : 0;

        if (state_test == 1 && last_state_test == 0)
        {
            printf("🔹 Appui sur BP_DEMANDE_TEST, envoi de notification...\n");
            xTaskNotify(xTaskGestionCasiersHandle, MODE_TEST, eSetValueWithOverwrite);
        }
        else if (state_appro == 1 && last_state_appro == 0)
        {
            printf("🔹 Appui sur BP_DEMANDE_APPRO, envoi de notification...\n");
            xTaskNotify(xTaskGestionCasiersHandle, MODE_APPRO, eSetValueWithOverwrite);
        }

        last_state_test = state_test;
        last_state_appro = state_appro;

        vTaskDelay(pdMS_TO_TICKS(100)); // Exécuter 10 fois par seconde
    }
}

void taskSimulateButtons(void *pvParameters)
{
    while (1)
    {
				vTaskDelay(pdMS_TO_TICKS(1000));

        // Simuler un appui sur BP_DEMANDE_APPRO (Passage en MODE_APPRO)
        printf("🔹 Simulation: Appui sur BP_DEMANDE_APPRO\n");
        GPIOB->BSRR = (1 << 0); // Simuler bouton PB0 pressé (LOW)
        vTaskDelay(pdMS_TO_TICKS(300));
        GPIOB->BSRR = (1 << 16); // Relâcher bouton PB0 (HIGH)

        // Attendre 5 secondes en MODE_APPRO
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Simuler un appui sur BP_DEMANDE_TEST (Passage en MODE_TEST)
        printf("🔹 Simulation: Appui sur BP_DEMANDE_TEST\n");
        GPIOA->BSRR = (1 << 2); // Simuler bouton PA2 pressé (LOW)
        vTaskDelay(pdMS_TO_TICKS(500));
        GPIOA->BSRR = (1 << 18); // Relâcher bouton PA2 (HIGH)

        // Attendre 5 secondes en MODE_TEST
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void taskGestionCasiers(void *pvParameters)
{
    uint32_t modeRecu;

    while (1)
    {
        // Attendre une notification venant de `taskScrutBoutons()`
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &modeRecu, portMAX_DELAY) == pdTRUE)
        {
            switch (modeRecu)
            {
            case MODE_TEST:
                printf("🟦 MODE_TEST: Casier 1 en bleu\n");
                num_case_resistance = 1;
                memset(ledStatus, 0, sizeof(ledStatus));
                ledStatus[0] = 1; // Allume le premier casier en bleu
                update_ledBuffer();
                start_ledTransmission();
                break;

            case MODE_APPRO:
                printf("🟩 MODE_APPRO: Tous en vert, Casier 1 en rouge\n");
                memset(ledStatus, 0xFF, sizeof(ledStatus)); // Tout en vert
                ledStatus[0] = 0xFE;                        // Casier 1 en rouge
                update_ledBuffer();
                start_ledTransmission();
                break;
            }
        }
    }
}

void taskMiseAJourLEDs(void *pvParameters)
{
    int i; // Déclarer i avant la boucle

    while (1)
    {
        for (i = 0; i < 60; i++)
        {
            if (etat_reed[i])
            {
                allumer_led_casier(i, ROUGE);
            }
            else
            {
                allumer_led_casier(i, VERT);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Mise à jour toutes les 100ms
    }
}

void taskGestionReedSwitches(void *pvParameters)
{
    static uint8_t ligneActuelle = 0;
    int i; // Déclarer i avant la boucle

    while (1)
    {
        // Lire les 5 capteurs sur la ligne actuelle
        for (i = 0; i < 5; i++)
        {
            int index = ligneActuelle * 5 + i;
            etat_reed[index] = (GPIOB->IDR & (1 << i)) ? 1 : 0;
        }

        // Changer de ligne
        ligneActuelle = (ligneActuelle + 1) % 12;

        vTaskDelay(pdMS_TO_TICKS(10)); // Attendre 10ms avant la ligne suivante
    }
}

void vClignoteurTask(void *pvParameters)
{
    PARAM_CLIGNOTEUR *param = (PARAM_CLIGNOTEUR *)pvParameters;

    init_GPIOx(param->port, param->pin, 0x3);

    while (1)
    {
        param->port->BSRR = (1 << param->pin); // Allumer la LED
        vTaskDelay(pdMS_TO_TICKS(param->dureeOn));

        param->port->BRR = (1 << param->pin); // Éteindre la LED
        vTaskDelay(pdMS_TO_TICKS(param->dureeOff));
    }
}

/* Instances des clignoteurs */
PARAM_CLIGNOTEUR ParamClignoteurA = {GPIOC, 9, 5, 4};
PARAM_CLIGNOTEUR ParamClignoteurB = {GPIOC, 10, 3, 2};
PARAM_CLIGNOTEUR ParamClignoteurC = {GPIOC, 11, 1, 2};

void vInit_myTasks(UBaseType_t uxPriority)
{
    xTaskCreate(vClignoteurTask, "ClignoteurA", configMINIMAL_STACK_SIZE, &ParamClignoteurA, uxPriority, NULL);
    xTaskCreate(vClignoteurTask, "ClignoteurB", configMINIMAL_STACK_SIZE, &ParamClignoteurB, uxPriority, NULL);
    xTaskCreate(vClignoteurTask, "ClignoteurC", configMINIMAL_STACK_SIZE, &ParamClignoteurC, uxPriority, NULL);
}
