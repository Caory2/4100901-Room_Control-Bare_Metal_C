/**
 ******************************************************************************
 * @file           : room_control.c
 * @author         : Sam C
 * @brief          : Room control driver for STM32L476RGTx
 ******************************************************************************
 */
#include "room_control.h"

#include "gpio.h"    // Para controlar LEDs y leer el botón (aunque el botón es por EXTI)
#include "systick.h" // Para obtener ticks y manejar retardos/tiempos
#include "uart.h"    // Para enviar mensajes
#include "tim.h"     // Para controlar el PWM



static uint8_t led_on = 0;
static uint32_t led_off_tick = 0;

void room_control_app_init(void) //init: se ejecuta 1 sola vez y esta fuera del loop 
{
    // Inicializar variables de estado si es necesario.
    // Por ejemplo, asegurar que los LEDs estén apagados al inicio
    gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET);
    gpio_write_pin(EXTERNAL_LED_PWM_PORT, EXTERNAL_LED_PWM_PIN, GPIO_PIN_RESET); //ver como funciona
    tim3_ch1_pwm_set_duty_cycle(50); // Establecer un duty cycle inicial para el PWM LED
}

void room_control_on_button_press(void)
{

    static uint32_t last_press_tick = 0;
    uint32_t now = systick_get_tick();

// TODO:Implementar anti-rebote si han pasado 50ms desde la ultima pulsacion
    if ((now - last_press_tick) >= DEBOUNCE) {
        last_press_tick = now;

        // Procesar la pulsación: encender LED externo por 3 segundos
        gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_SET);
        led_on = 1;
        led_off_tick = now + 3000; // 3 segundos

        uart2_send_string("Boton B1: Presionado LED externo ON\r\n");
    }
}

// Apagar el LED externo después de 3 segundos (llama esto periódicamente, por ejemplo desde SysTick)
void room_control_periodic_1ms(void)
{
    uint32_t now = systick_get_tick();

    if (led_on && (now >= led_off_tick)) {
        gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET);
        led_on = 0;
        uart2_send_string("LED externo: OFF\r\n");
    }
}
// Función a ser llamada por USART2_IRQHandler cuando se recibe un carácter.
    // TODO: Procesar el carácter para realizar acciones
    // Ejemplo: si recibe 'h' o 'H', encender el LED PWM al 100%.
    //          si recibe 'l' o 'L', apagar el LED PWM (0%).
    //          si recibe 't', hacer toggle del LED ON/OFF.
void room_control_on_uart_receive(char received_char)
{
    // Eco del carácter recibido
    uart2_send_char(received_char);

    if (received_char == 'h' || received_char == 'H') {
        tim3_ch1_pwm_set_duty_cycle(100);
        uart2_send_string("\r\nPWM LED: 100%\r\n");
    }
    else if (received_char == 'l' || received_char == 'L') {
        tim3_ch1_pwm_set_duty_cycle(0);
        uart2_send_string("\r\nPWM LED: 0%\r\n");
    }
    else if (received_char == 't' || received_char == 'T') {
        if (led_on) {
            gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_RESET);
            led_on = 0;
            uart2_send_string("\r\nLED externo: OFF\r\n");
        } else {
            gpio_write_pin(EXTERNAL_LED_ONOFF_PORT, EXTERNAL_LED_ONOFF_PIN, GPIO_PIN_SET);
            led_on = 1;
            led_off_tick = systick_get_tick() + 3000;
            uart2_send_string("\r\nLED externo: ON (3s)\r\n");
        }
    }
}
