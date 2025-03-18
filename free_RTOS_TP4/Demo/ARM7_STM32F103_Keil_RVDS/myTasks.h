
#ifndef MYTASKS_LED_H
#define MYTASKS_LED_H

#include "FreeRTOS.h"
#include "task.h"

void vInit_myTasks( UBaseType_t uxPriority );


// Définition des broches des boutons (à ajuster selon ton hardware)
#define BP_DEMANDE_TEST  GPIO_Pin_2   // PA2
#define BP_DEMANDE_APPRO GPIO_Pin_1   // PA1

// Définition des couleurs LED
#define BLEU  0x0000FF
#define ROUGE 0xFF0000
#define VERT  0x00FF00

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    TickType_t dureeOn;
    TickType_t dureeOff;
} PARAM_CLIGNOTEUR;


// Définition des modes
typedef enum {
    MODE_EFFACE,
    MODE_TEST,
    MODE_APPRO
} ModeSysteme;

// Variables globales
extern volatile ModeSysteme modeActuel;
extern unsigned char etat_reed[60]; // Tableau d'état des capteurs REED
extern volatile int num_case_resistance; // Numéro du casier à tester

// Déclarations des tâches
void taskScrutBoutons(void *pvParameters);
void taskGestionCasiers(void *pvParameters);
void taskMiseAJourLEDs(void *pvParameters);
void taskGestionReedSwitches(void *pvParameters);
void allumer_led_casier(int casier, uint32_t couleur);


//extern void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet);

void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet);

#endif
