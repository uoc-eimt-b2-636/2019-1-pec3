/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* Launchpad, Wi-Fi and Sensors includes */
#include "msp432_launchpad_board.h"
#include "cc3100_boosterpack.h"
#include "edu_boosterpack_sensors.h"
#include "msp432_launchpad_temperature.h"

/* TI Graphics library */
#include "st7735.h"
#include "st7735_msp432.h"
#include "grlib.h"

/* ARM math library */
#include "arm_math.h"

/*----------------------------------------------------------------------------*/

#define SENSORS_TASK_PRIORITY   ( 2 )
#define SCREEN_TASK_PRIORITY    ( 1 )

#define SENSORS_STACK_SIZE      ( 1024 )
#define SCREEN_STACK_SIZE       ( 1024 )

/*----------------------------------------------------------------------------*/

Graphics_Context g_sContext;

float ambient_temp;
float ambient_light;
int16_t acceleration_x;
int16_t acceleration_y;
int16_t acceleration_z;

char string_temp[20];
char string_light[20];
char string_x[20];
char string_y[20];
char string_z[20];

SemaphoreHandle_t xSemaphore;

/*----------------------------------------------------------------------------*/

static void SensorsTask(void *pvParameters) {
    /* Initialize sensors */
    edu_boosterpack_sensors_init();

    while(true)
    {
        /* Turn green LED on */
        led_green_on();

        ambient_temp = 0.0f;
        ambient_light = 0.0f;
        acceleration_x = 0;
        acceleration_y = 0;
        acceleration_z = 0;

        /* Read temperature */
        edu_boosterpack_sensors_temperature_read(&ambient_temp);

        /* Read light */
        edu_boosterpack_sensors_light_read(&ambient_light);

        /* Read acceleration */
        edu_boosterpack_sensors_acceleration_read(ambient_temp, &acceleration_x, &acceleration_y, &acceleration_z);

        /* Try to take semaphore to read sensors */
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            int tmpInt1, tmpInt2;
            float tmpFrac;

            /* Write temperature reading */
            tmpInt1 = ambient_temp;
            tmpFrac = ambient_temp - tmpInt1;
            tmpInt2 = trunc(tmpFrac * 10);
            sprintf(string_temp, "%2d.%01d", tmpInt1, tmpInt2);

            /* Write light reading */
            tmpInt1 = ambient_light;
            tmpFrac = ambient_light - tmpInt1;
            tmpInt2 = trunc(tmpFrac * 10);
            sprintf(string_light, "%3d.%01d", tmpInt1, tmpInt2);

            /* Write acceleration readings */
            if (acceleration_x <= 0) {
                sprintf(string_x, "- %4d", abs(acceleration_x));
            } else {
                sprintf(string_x, "+ %4d", abs(acceleration_x));
            }

            if (acceleration_y <= 0) {
                sprintf(string_y, "- %4d", abs(acceleration_y));
            } else {
                sprintf(string_y, "+ %4d", abs(acceleration_y));
            }

            if (acceleration_z <= 0) {
                sprintf(string_z, "- %4d", abs(acceleration_z));
            } else {
               sprintf(string_z, "+ %4d", abs(acceleration_z));
            }

            /* Give the semaphore */
            xSemaphoreGive(xSemaphore);
        }

        /* Turn green LED off */
        led_green_off();

        /* Sleep for 100 ms */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void ScreenTask(void *pvParameters) {
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);

    Graphics_drawString(&g_sContext,
                    "Ambient temp.:",
                    AUTO_STRING_LENGTH,
                    10,
                    20,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "Ambient light:",
                    AUTO_STRING_LENGTH,
                    10,
                    50,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "Acceleration:",
                    AUTO_STRING_LENGTH,
                    10,
                    80,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "degC",
                    AUTO_STRING_LENGTH,
                    80,
                    30,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "lux",
                    AUTO_STRING_LENGTH,
                    80,
                    60,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "x = ",
                    AUTO_STRING_LENGTH,
                    10,
                    90,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "mG",
                    AUTO_STRING_LENGTH,
                    80,
                    90,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "y = ",
                    AUTO_STRING_LENGTH,
                    10,
                    100,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "mG",
                    AUTO_STRING_LENGTH,
                    80,
                    100,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "z = ",
                    AUTO_STRING_LENGTH,
                    10,
                    110,
                    OPAQUE_TEXT);

    Graphics_drawString(&g_sContext,
                    "mG",
                    AUTO_STRING_LENGTH,
                    80,
                    110,
                    OPAQUE_TEXT);

    while(true)
    {
        /* Try to take semaphore to write to screen */
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            /* Turn red LED on */
            led_red_on();

            Graphics_drawString(&g_sContext,
                                (int8_t *)string_temp,
                                AUTO_STRING_LENGTH,
                                40,
                                30,
                                OPAQUE_TEXT);

            Graphics_drawString(&g_sContext,
                                (int8_t *)string_light,
                                AUTO_STRING_LENGTH,
                                40,
                                60,
                                OPAQUE_TEXT);

            Graphics_drawString(&g_sContext,
                                (int8_t *)string_x,
                                AUTO_STRING_LENGTH,
                                40,
                                90,
                                OPAQUE_TEXT);

            Graphics_drawString(&g_sContext,
                                (int8_t *)string_y,
                                AUTO_STRING_LENGTH,
                                40,
                                100,
                                OPAQUE_TEXT);

            Graphics_drawString(&g_sContext,
                                (int8_t *)string_z,
                                AUTO_STRING_LENGTH,
                                40,
                                110,
                                OPAQUE_TEXT);

            /* Turn red LED on */
            led_red_off();

            /* Give the semaphore */
            xSemaphoreGive(xSemaphore);
        }

        /* Sleep for 50 ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*
 * Application's entry point
 */
int main(int argc, char** argv)
{
    int32_t retVal = 0;

    /* Initialize the board */
    board_init();

    /* Create mutex */
    xSemaphore = xSemaphoreCreateMutex();
    if(xSemaphore == NULL)
    {
        /* Turn red LED on */
        led_red_on();

        /* Lock */
        while(1);
    }

    /* Create main task */
    retVal = xTaskCreate(SensorsTask,
                         "SensorsTask",
                         SENSORS_STACK_SIZE,
                         NULL,
                         SENSORS_TASK_PRIORITY,
                         NULL);
    if (retVal < 0)
    {
        /* Turn red LED on */
        led_red_on();

        /* Lock */
        while(1);
    }

    /* Create screen task */
    retVal = xTaskCreate(ScreenTask,
                         "ScreenTask",
                         SCREEN_STACK_SIZE,
                         NULL,
                         SCREEN_TASK_PRIORITY,
                         NULL);
    if (retVal < 0)
    {
        /* Turn red LED on */
        led_red_on();

        /* Lock */
        while(1);
    }

    /* Start the task scheduler */
    vTaskStartScheduler();

    return 0;
}

/*----------------------------------------------------------------------------*/
