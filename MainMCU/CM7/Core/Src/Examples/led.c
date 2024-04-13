#include "../../Inc/Examples/led.h"

#define GPIOAEN         (1U<<0) //setting position 0 to 1
#define LED_PIN         (1U<<5)

void led_init(void) {
    //Enable clock access to led port (Port A)
    RCC->AHB4ENR |= GPIOAEN;
    //Doing an OR is done so that you do not override the entire register
    //0b 0000 0000 0000 0000 0000 0000 0000 0000 
    //GPIO Port A: 0b 0000 0000 0000 0000 0000 0000 0000 0001
    //GPIO Port D:  0b 0000 0000 0000 0000 0000 0000 0000 1000
    //To have both enabled, simply OR it would be better
    //Note: add would make random registers enabled
    //Set bit0 = (1u<<0)

    //Set LED as an output
    GPIOA->MODER |= (1U<<10);
    GPIOA->MODER &= ~(1U<<11);
}

/**
 * sets LED on
*/
void led_on(void) {
    GPIOA->ODR |= LED_PIN;
}


/**
 * sets LED off
*/
void led_off(void) {
    GPIOA->ODR &= ~LED_PIN; //boolean negation
}