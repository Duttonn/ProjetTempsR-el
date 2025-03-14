
#ifndef MYTASKS_LED_H
#define MYTASKS_LED_H

void vInit_myTasks( UBaseType_t uxPriority );

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    TickType_t dureeOn;
    TickType_t dureeOff;
} PARAM_CLIGNOTEUR;



#endif
