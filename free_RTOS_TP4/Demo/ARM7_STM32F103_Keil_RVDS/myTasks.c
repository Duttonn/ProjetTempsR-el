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




// Variables globales pour ADC
volatile uint16_t adc_value = 0;
volatile uint8_t adc_ready = 0;
extern uint8_t ledStatus[60];
volatile uint8_t casier_debug = 255;  // Variable globale pour le Logic Analyzer
volatile uint8_t casier = 255;
volatile int i;


#define ADC_CHANNEL 0  // Utiliser directement 0 pour PA0
#define PIN_CHOIX_TALON 5 // GPIO PB5 pour activer RES2


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
        uint8_t state_appro = (GPIOB->IDR & (1 << 0)) ? 1 : 0;

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
        vTaskDelay(pdMS_TO_TICKS(150));
        GPIOB->BSRR = (1 << 16); // Relâcher bouton PB0 (HIGH)

        // Attendre 5 secondes en MODE_APPRO
        vTaskDelay(pdMS_TO_TICKS(500));

        // Simuler un appui sur BP_DEMANDE_TEST (Passage en MODE_TEST)
        printf("🔹 Simulation: Appui sur BP_DEMANDE_TEST\n");
        GPIOA->BSRR = (1 << 2); // Simuler bouton PA2 pressé (LOW)
        vTaskDelay(pdMS_TO_TICKS(150));
        GPIOA->BSRR = (1 << 18); // Relâcher bouton PA2 (HIGH)

        // Attendre 5 secondes en MODE_TEST
        vTaskDelay(pdMS_TO_TICKS(1000));
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

const uint16_t SEUILS1[] = {
    3850,3804,3752,3691,3618,3532,3432,3320,3183,3033,
    2875,2695,2504,2322,2143,1959,1772,1583,1401,1231,
    1065,913,785,664,559,476,406,345,291,244,
    203
};

const uint16_t SEUILS2[] = {
    3898,3859,3810,3752,3686,3605,3511,3414,3307,3185,
    3048,2893,2723,2545,2346,2141,1946,1741,1542,1369,
    1214,1065,927,797,681,580,487,406,341,283,
    234
};

const uint16_t CASIERS[] = {
    0,1,2,3,4,5,6,7,8,9,
    10,11,12,13,14,15,16,17,18,19,
    20,21,22,23,24,25,26,27,28,29,
    30,31,32,33,34,35,36,37,38,39,
    40,41,42,43,44,45,46,47,48,49,
    50,51,52,53,54,55,56,57,58,59,
};

void Set_Transistor(uint8_t state);
void Start_ADC_Conversion(void);
void update_ledBuffer(void);
void start_ledTransmission(void);

	
void vClignoteurTask(void *pvParameters) {
    PARAM_CLIGNOTEUR *param = (PARAM_CLIGNOTEUR *) pvParameters;


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



void vInit_myTasks(UBaseType_t uxPriority) {
    //xTaskCreate(vClignoteurTask, "ClignoteurA", configMINIMAL_STACK_SIZE, &ParamClignoteurA, uxPriority, NULL);
    //xTaskCreate(vClignoteurTask, "ClignoteurB", configMINIMAL_STACK_SIZE, &ParamClignoteurB, uxPriority, NULL);
    //xTaskCreate(vClignoteurTask, "ClignoteurC", configMINIMAL_STACK_SIZE, &ParamClignoteurC, uxPriority, NULL);
	  xTaskCreate(vTask_Mesure_Resistance, "MesureRes", configMINIMAL_STACK_SIZE + 100, NULL, uxPriority, NULL);


}


// === Activer/Désactiver RES2 via PB5 ===
void Set_Transistor(uint8_t state) {
    if (state)
        GPIOB->BSRR = (1 << PIN_CHOIX_TALON); // Activer RES2
    else
        GPIOB->BSRR = (1 << (PIN_CHOIX_TALON + 16)); // Désactiver RES2
}

// === Démarrer la conversion ADC ===
void Start_ADC_Conversion(void) {
    ADC1->CR2 |= (1 << 22);  // Déclencher la conversion (SWSTART)
}

/*
void Init_ADC(void) {
    // Activer horloge GPIOA et ADC1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // Configurer PA0 (canal 0) en mode analogique (entr�e flottante)
    GPIOA->CRL &= ~(0xF << (0 * 4));  // Remet � 0 les 4 bits du pin PA0

    // Activer l'interruption ADC sur fin de conversion
    ADC1->CR1 = ADC_CR1_EOCIE;

    // ADC ON - 1�re activation
    ADC1->CR2 |= ADC_CR2_ADON;
    for (i = 0; i < 1000; i++); // Petit d�lai pour stabiliser

    // ADC ON - 2e activation obligatoire (hardware requirement)
    ADC1->CR2 |= ADC_CR2_ADON;

    // Mode conversion continue
    ADC1->CR2 |= ADC_CR2_CONT;

    // S�lection du canal 0 en premi�re position dans la s�quence
    ADC1->SQR3 = 0;

    // Activer l�interruption ADC dans le contr�leur NVIC
    NVIC_EnableIRQ(ADC1_2_IRQn);

    // D�marrer la premi�re conversion (il continue ensuite tout seul)
    ADC1->CR2 |= ADC_CR2_SWSTART;
}*/



   // NVIC_EnableIRQ((IRQn_Type)18);

void Init_ADC(void) {
  int i;  
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;

    // PA0 en entr�e analogique
    GPIOA->CRL &= ~(0xF << (0 * 4));

    ADC1->CR1 = ADC_CR1_EOCIE; // Interruption fin de conversion

    ADC1->CR2 |= ADC_CR2_ADON; // 1�re activation
    for (i = 0; i < 1000; i++); // Petit d�lai
    ADC1->CR2 |= ADC_CR2_ADON; // 2�me activation

    ADC1->SQR3 = 0; // Canal 0
    NVIC_EnableIRQ(ADC1_2_IRQn);
}



void ADC1_2_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
        adc_value = ADC1->DR;             // 1. Lire la donn�e
        ADC1->SR &= ~ADC_SR_EOC;          // 2. Effacer le flag
        adc_ready = 1;                    // 3. Signaler � la t�che
    }
}

/*
void vTask_Mesure_Resistance(void *pvParameters) {
    uint16_t valeur = 0;
    int i;

    while (1) {
        // === Mesure initiale avec RES1 seule ===
        Set_Transistor(0);

        //adc_ready = 0;                 // 1. On se pr�pare � recevoir une nouvelle conversion
			  Start_ADC_Conversion();
        while (!adc_ready);           // 2. On attend que l�interruption la remplisse
        valeur = adc_value;           // 3. On r�cup�re la valeur

        // === Si la r�sistance est faible (valeur �lev�e) ===
        if (valeur > 3850) {
						volatile int j;
            Set_Transistor(1);        // Activer RES2
						for (j = 0; j < 10000; j++); //a voir 
            adc_ready = 0;  //a voir 
					  Start_ADC_Conversion();
            while (!adc_ready);
            valeur = adc_value;

            // Recherche dans SEUILS2 (casier 0 � 29)
            for (i = 0; i < sizeof(SEUILS2)/sizeof(uint16_t) - 1; i++) {
                if (valeur <= SEUILS2[i] && valeur >= SEUILS2[i + 1]) {
                    casier = CASIERS[i];
                    break;
                }
            }

            Set_Transistor(0);        // D�sactiver RES2 apr�s mesure

        } else {
            // Recherche dans SEUILS1 (casier 30 � 59)
            for (i = 0; i < sizeof(SEUILS1)/sizeof(uint16_t) - 1; i++) {
                if (valeur <= SEUILS1[i] && valeur >= SEUILS1[i + 1]) {
                    casier = CASIERS[i + 30];
                    break;
                }
            }
        }

        // === Debug et affichage sur LEDS ===
        casier_debug = casier;

        if (casier < 60) {
            uint8_t byteIndex = casier / 8;
            uint8_t bitIndex = casier % 8;
            ledStatus[byteIndex] |= (1 << bitIndex);
        }

        update_ledBuffer();
        start_ledTransmission();

        vTaskDelay(pdMS_TO_TICKS(1000)); // Attente 1s avant la prochaine mesure
    }
}*/

void vTask_Mesure_Resistance(void *pvParameters) {
    uint16_t valeur = 0;
    int i;

    while (1) {
        Set_Transistor(0);
        Start_ADC_Conversion();
        while (!adc_ready);
        valeur = adc_value;

        // === Si pas de mesure valable ===
        if (valeur == 0) {
            casier = 255;
            casier_debug = casier;

            // �teindre toutes les LEDs
            for (i = 0; i < sizeof(ledStatus); i++) {
                ledStatus[i] = 0;
            }

            //update_ledBuffer();
            //start_ledTransmission();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // === Choix de la r�sistance parall�le si besoin ===
        if (valeur > 3850) {
            Set_Transistor(1);
            Start_ADC_Conversion();
            while (!adc_ready);
            valeur = adc_value;

            for (i = 0; i < sizeof(SEUILS2)/sizeof(uint16_t) - 1; i++) {
                if (valeur <= SEUILS2[i] && valeur >= SEUILS2[i + 1]) {
                    casier = CASIERS[i];
                    break;
                }
            }

            Set_Transistor(0);
        } else {
            for (i = 0; i < sizeof(SEUILS1)/sizeof(uint16_t) - 1; i++) {
                if (valeur <= SEUILS1[i] && valeur >= SEUILS1[i + 1]) {
                    casier = CASIERS[i + 30];
                    break;
                }
            }
        }

        casier_debug = casier;

        // === Affichage sur LEDs : tableau de 60 bits ===
        for (i = 0; i < sizeof(ledStatus); i++) {
            ledStatus[i] = 0;  // Tout �teindre
        }




        if (casier < 60) {
            ledStatus[casier] = 1;  // Allume seulement ce casier
        }

        update_ledBuffer();
        start_ledTransmission();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}





