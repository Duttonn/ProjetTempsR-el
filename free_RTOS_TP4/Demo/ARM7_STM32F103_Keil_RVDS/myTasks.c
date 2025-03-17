#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"


/* le fichier .h du TP  */
#include "myTasks.h"

void init_GPIOx(GPIO_TypeDef *GPIOx, uint8_t num_bit, uint8_t quartet);


void vClignoteurTask(void *pvParameters) {
    PARAM_CLIGNOTEUR *param = (PARAM_CLIGNOTEUR *) pvParameters;

    init_GPIOx(param->port, param->pin, 0x3);

    while (1) {
        param->port->BSRR = (1 << param->pin);  // Allumer la LED
        vTaskDelay(pdMS_TO_TICKS(param->dureeOn));

        param->port->BRR = (1 << param->pin);  // �teindre la LED
        vTaskDelay(pdMS_TO_TICKS(param->dureeOff));
    }
}

/* Instances des clignoteurs */
PARAM_CLIGNOTEUR ParamClignoteurA = {GPIOC, 9, 5, 4};
PARAM_CLIGNOTEUR ParamClignoteurB = {GPIOC, 10, 3, 2};
PARAM_CLIGNOTEUR ParamClignoteurC = {GPIOC, 11, 1, 2};

void vInit_myTasks(UBaseType_t uxPriority) {
    xTaskCreate(vClignoteurTask, "ClignoteurA", configMINIMAL_STACK_SIZE, &ParamClignoteurA, uxPriority, NULL);
    xTaskCreate(vClignoteurTask, "ClignoteurB", configMINIMAL_STACK_SIZE, &ParamClignoteurB, uxPriority, NULL);
    xTaskCreate(vClignoteurTask, "ClignoteurC", configMINIMAL_STACK_SIZE, &ParamClignoteurC, uxPriority, NULL);
}
