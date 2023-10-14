#ifndef bsp_t4_h
#define bsp_t4_h

#include "driver/gpio.h"
#include "unistd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

// Estructura para la configuración de GPIO
typedef struct {
    gpio_pullup_t pull_up_en;       /*!< GPIO pull-up                                         */
    gpio_pulldown_t pull_down_en;   /*!< GPIO pull-down                                       */
    gpio_int_type_t intr_type;      /*!< GPIO interrupt type                                  */
    gpio_num_t gpio_pin;
    gpio_mode_t mode;
} BSP_GPIO_Config;

typedef struct {
	uint8_t out[3];
	TimerHandle_t timer;
} BSP_TIMER_Param;

// Funciones de inicialización y configuración
void BSP_GPIO_Init(BSP_GPIO_Config* config);
void BSP_GPIO_LUM(gpio_num_t* gpio_num, uint8_t* out);
uint8_t* BSP_GPIO_Level(gpio_num_t* gpio_num);
void BSP_CONS_Estados(uint8_t* out, TimerHandle_t timer, bool flag);
void timer_callback(TimerHandle_t timer);


#endif // BSP_H
