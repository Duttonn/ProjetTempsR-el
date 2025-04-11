
#ifndef MYTASKS_LED_H
#define MYTASKS_LED_H

#include "FreeRTOS.h"
#include "task.h"

void vInit_myTasks(UBaseType_t uxPriority);

// D�finition des broches des boutons (� ajuster selon ton hardware)
#define BP_DEMANDE_TEST GPIO_Pin_2  // PA2
#define BP_DEMANDE_APPRO GPIO_Pin_1 // PA1

// D�finition des couleurs LED
#define BLEU 0x0000FF
#define ROUGE 0xFF0000
#define VERT 0x00FF00

extern uint8_t ledStatus[60]; // Statut des LEDs (tableau de 8 octets pour 60 LEDs)
extern TaskHandle_t xTaskGestionCasiersHandle;

void update_ledBuffer(void);
void start_ledTransmission(void);
void taskSimulateButtons(void *pvParameters);

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    TickType_t dureeOn;
    TickType_t dureeOff;
} PARAM_CLIGNOTEUR;

// D�finition des modes
typedef enum
{
    MODE_EFFACE,
    MODE_TEST,
    MODE_APPRO
} ModeSysteme;

// Variables globales
extern volatile ModeSysteme modeActuel;
extern unsigned char etat_reed[60];      // Tableau d'�tat des capteurs REED
extern volatile int num_case_resistance; // Num�ro du casier � tester

// D�clarations des t�ches
void taskScrutBoutons(void *pvParameters);
void taskGestionCasiers(void *pvParameters);
void taskMiseAJourLEDs(void *pvParameters);
void taskGestionReedSwitches(void *pvParameters);
void allumer_led_casier(int casier, uint32_t couleur);

// extern void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet);

void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet);

#endif
