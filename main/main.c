/*
 * Nombre de Archivo: main.c
 *
 * Descripción: Prototipo de Sistema Embebido Para Control de Luminarias en un Aula Escolar
 * 				Archivo Main de ESP32
 * 				Controla mediante tareas de FREERTOS Timer, GPIO/UART, Perceptron, Aplicación Principal.
 *
 * TECNM - Campus Chihuahua
 *
 *  Diseño De Sistemas Embebidos Inteligentes
 *
 *  Tarea 4: Diseño De Sistema Embebido
 *
 * Integrantes:
 *   Axel Gay Díaz							19060718
 *   Carlos Alberto González Vazquez		19060770
 *   Luis Octavio Méndez Valles				19060757
 *   Hiram Ochoa Sáenz						19060760
 *
 * Docente:
 *   Dr. Juan Alberto Ramírez Quintana
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "bsp_t4.h"

/* Definición de botones. */
#define BSYS_ON    GPIO_NUM_4
#define BLIGHT1    GPIO_NUM_14
#define BLIGHT2    GPIO_NUM_33

/* Definición de salidas. */
#define LUM1 	   GPIO_NUM_18
#define LUM2	   GPIO_NUM_19

QueueHandle_t botones_queue;
QueueHandle_t salidas_queue;
TaskHandle_t Task_perceptron;
TaskHandle_t Task_print;
TimerHandle_t timer;

bool myFlag = 1;

static void perceptron(void* arg)
{
    uint8_t botones[3];
    float w[3][4] = {
					 {1.5, 0.5, 0.5, -2.0},
					 {1.0, 0.5, 0.0, -1.5},
					 {1.0, 0.5, 0.5, -2.0}
    									  };
    uint8_t out[3];
    for(;;) {
        if (xQueueReceive(botones_queue, &botones, portMAX_DELAY)){
        	for (int i = 0; i < 3; i++) {
    			double g = 0;
    			for (int d = 0; d < 3; d++) {
    				g += w[i][d] * botones[d];
    			}
    			g += w[i][3]; // w[dim] es equivalente a wo

    			out[i] = (g >= 0) ? 1 : 0; // Función de activación
        	}
        	if (out[2])
        		myFlag = !myFlag;
        	xQueueSend(salidas_queue, out, portMAX_DELAY);
        }
    }
}

static void print(void* arg){
	BSP_TIMER_Param* timer_p = (BSP_TIMER_Param*)arg;
	uint8_t salidas[3];
	gpio_num_t gpio[2] = {LUM1, LUM2};
	uint8_t salida1_pas = 0, salida2_pas = 0, salida3_pas = 0;
	for(;;) {
		if (xQueueReceive(salidas_queue, &salidas, portMAX_DELAY)){
			timer_p->out[0] = salidas[0];
			timer_p->out[1] = salidas[1];
			timer_p->out[2] = salidas[2];
			if (salidas[2])
				xTimerStart(timer, NULL);
			BSP_GPIO_LUM(gpio, salidas);
			if (salidas[0] != salida1_pas || salidas[1] != salida2_pas || salidas[2] != salida3_pas || xTimerIsTimerActive(timer)){
				salida1_pas = salidas[0];
				salida2_pas = salidas[1];
				salida3_pas = salidas[2];
				BSP_CONS_Estados(salidas, timer, myFlag);
			}
			vTaskDelay(50 / portTICK_RATE_MS);
		}
	}
}

void app_main(void)
{
	BSP_TIMER_Param TimerP = {};
	TimerP.timer = timer;

	// Configuración luminarias
	BSP_GPIO_Config io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.pull_down_en = false;
	io_conf.pull_up_en = false;
	io_conf.gpio_pin = LUM1;
	io_conf.mode = GPIO_MODE_OUTPUT;
	BSP_GPIO_Init(&io_conf);
	io_conf.gpio_pin = LUM2;
	BSP_GPIO_Init(&io_conf);

	// Configuración botones
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.gpio_pin = BSYS_ON;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = true;
    BSP_GPIO_Init(&io_conf);
    io_conf.gpio_pin = BLIGHT1;
    BSP_GPIO_Init(&io_conf);
    io_conf.gpio_pin = BLIGHT2;
    BSP_GPIO_Init(&io_conf);

	botones_queue = xQueueCreate(1, 3 * sizeof(uint8_t));
	salidas_queue = xQueueCreate(1, 3 * sizeof(uint8_t));

	timer = xTimerCreate("mi_timer", pdMS_TO_TICKS(5000), pdFALSE, &TimerP, &timer_callback);

	xTaskCreate(perceptron, "tarea_perceptron", 2048, NULL, 8, &Task_perceptron);
	xTaskCreate(print, "tarea_imprimir", 2048, &TimerP, 8, &Task_print);

	gpio_num_t gpio[3] = {BSYS_ON, BLIGHT1, BLIGHT2};
	while(1) {
		uint8_t *entradas = BSP_GPIO_Level(gpio);

		xQueueSend(botones_queue, entradas, portMAX_DELAY);
		if (entradas[0] && entradas[1] && entradas[2])
			vTaskDelay(100 / portTICK_RATE_MS);
	}
}
