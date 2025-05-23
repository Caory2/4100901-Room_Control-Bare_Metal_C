/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sam C
 * @brief          : Main program body
 ******************************************************************************
 */
#include "gpio.h"
#include "systick.h"
#include "nvic.h"
#include "uart.h"
#include "tim.h"
#include "room_control.h"

void heartbeat_led_toggle(void)
{
    static uint32_t last_tick = 0;
    if (systick_get_tick() - last_tick >= 500) { // Cambia cada 500 ms
        gpio_toggle_pin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        last_tick = systick_get_tick();
    }
}

/**
 * @brief Función principal del programa.
 *        Configura los periféricos y entra en un bucle infinito.
 *        El LED de heartbeat parpadea cada 500 ms.
*/
int main(void)
{
    // Inicialización de SysTick
    systick_init_1ms(); // Utiliza SYSCLK_FREQ_HZ (ej. 4MHz) de rcc.h

    // LED Heartbeat
    gpio_setup_pin(GPIOA, HEARTBEAT_LED_PIN, GPIO_MODE_OUTPUT, 0);

    // LED Externo ON/OFF
    gpio_setup_pin(GPIOA, EXTERNAL_LED_ONOFF_PIN, GPIO_MODE_OUTPUT, 0);

    // LED PWM
    gpio_setup_pin(GPIOA, 6, GPIO_MODE_AF, 2); // (AF2 para TIM3_CH1 en PA6)

    // Botón B1
    gpio_setup_pin(GPIOC, 13, GPIO_MODE_INPUT, 0);
    nvic_exti_pc13_button_enable();

    // USART2
    uart2_init(115200);
    nvic_usart2_irq_enable();

    // TIM3 Canal 1 para PWM
    tim3_ch1_pwm_init(1000); // ej. 1000 Hz
    tim3_ch1_pwm_set_duty_cycle(40); // ej. 50%

    // Inicialización de la Lógica de la Aplicación (room_control)
    room_control_app_init();
    uint8_t prev_button_state = 1; // Estado anterior del botón (1 = no presionado, 0 = presionado)
    uint32_t last_press_tick = 0;


    // Mensaje de bienvenida o estado inicial (puede estar en room_control_app_init o aquí)


    uart2_send_string("\r\nSistema Inicializado. Esperando eventos...\r\n");
    while (1) {
        
        heartbeat_led_toggle();

        // Leer el estado actual del botón (ajusta si tu función es diferente)
        uint8_t button_state = gpio_read_pin(GPIOC, 13);

        // Detectar flanco de bajada (presionado) con anti-rebote
        if (prev_button_state == 1 && button_state == 0) {
            uint32_t now = systick_get_tick();
            if ((now - last_press_tick) >= DEBOUNCE) {
                last_press_tick = now;
                room_control_on_button_press(); // Llama a tu función de aplicación
            }
        }
        prev_button_state = button_state;

    }
}

    
