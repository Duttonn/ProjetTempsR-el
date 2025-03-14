
/* Standard includes. */
#include <stdlib.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
// pour initialisation facile des PATTES du proc
#include "def_type_gpio.h"
#include "myTasks.h"

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )


 // Configuration minimal du processeur pour avoir les 8 Leds du Port2 disponible
 
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
void gestion_abort(void)
{


}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{ static uint32_t oscille_plantage = 1<<13; // la pile du main est massacrée par l'OS, on ne peut pas laisser la variable sur la pile, d où le static
 // Configuration minimal du processeur pour avoir les 8 Leds du Port2 disponible
	prvSetupHardware();

	/* initialisation des taches */
	vInit_myTasks( mainLED_TASK_PRIORITY );

	// toutes les taches ont été démarrées - Demarrer le scheduler.
  // Les taches tournent en USER/SYSTEM mode et le Scheduler tourne en Superviseur mode
	// Le processeur doit être en SUPERVISEUR quand vTaskStartScheduler est appelé
  // mais user privileged suffit en fait...	
	
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */

	for( ;; )
	{
		 GPIOB->BSRR = oscille_plantage;oscille_plantage ^= 0x00010001<<13;
	}
}

void init_gpio(void)
{//sera clignoteur 2
 init_GPIOx(GPIOA,4, GPIO_MODE_OUTPUT_PP_50MHz );  // sera clignoteur 1
 init_GPIOx(GPIOA,5, GPIO_MODE_OUTPUT_PP_50MHz );  // sera clignoteur 2
 init_GPIOx(GPIOA,6, GPIO_MODE_OUTPUT_PP_50MHz );  // sera clignoteur 3
 init_GPIOx(GPIOA,7, GPIO_MODE_OUTPUT_PP_50MHz );  // sera clignoteur 4
 init_GPIOx(GPIOB,10, GPIO_MODE_OUTPUT_PP_50MHz ); // sera espion IT_TICKS
 init_GPIOx(GPIOB,11, GPIO_MODE_OUTPUT_PP_50MHz ); // sera espion TASK_IDLE
 init_GPIOx(GPIOB,12, GPIO_MODE_OUTPUT_PP_50MHz ); // sera espion vTaskSwitchContext
 init_GPIOx(GPIOB,13, GPIO_MODE_OUTPUT_PP_50MHz ); // sera espion plantage (pas de chance)
}

static void prvSetupHardware( void )
{
init_gpio();	
}
