#include <stdio.h>
#include "bsp_t4.h"

void BSP_GPIO_Init(BSP_GPIO_Config* config) {
    // Verificar si la configuración es válida
    if (config == 0) {
        // Manejar el error o salir de la función si la configuración no es válida
        return;
    }
    // Configurar el pin GPIO según la configuración proporcionada
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = (1ULL << config->gpio_pin);
    gpioConfig.mode = config->mode;
    gpioConfig.pull_up_en = config->pull_up_en;
    gpioConfig.pull_down_en = config->pull_down_en;
    gpioConfig.intr_type = config->intr_type;
    gpio_config(&gpioConfig);
}

void BSP_GPIO_LUM(gpio_num_t* gpio_num, uint8_t* out){
	gpio_set_level(gpio_num[0], out[0]);
	gpio_set_level(gpio_num[1], out[1]);
}

uint8_t* BSP_GPIO_Level(gpio_num_t* gpio_num){
	static uint8_t botones[3];

	botones[0] = !gpio_get_level(gpio_num[0]);
	botones[1] = !gpio_get_level(gpio_num[1]);
	botones[2] = !gpio_get_level(gpio_num[2]);

	return botones;
}
