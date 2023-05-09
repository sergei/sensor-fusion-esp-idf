#include <sys/cdefs.h>
/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file main_agm01_freertos_two_tasks.c
    \brief FreeRTOS (two task) implementation of sensor fusion on FRDM-K64F.

    This file shows one recommended way to incorporate sensor fusion capabilities
    into a FreeRTOS project.
*/

/* FreeRTOS kernel includes. */
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"


// Sensor Fusion Headers
#include "sensor_fusion.h" // top level magCal and sensor fusion interfaces
#include "control.h"       // Command/Streaming interface - application specific
#include "status.h"        // Sta:tus indicator interface - application specific
#include "../drivers/drivers.h"       // NXP sensor drivers OR customer-supplied drivers

static const char *TAG = "sensor_fusion_main_two";

// Global data structures
SensorFusionGlobals sfg;           ///< This is the primary sensor fusion data structure
ControlSubsystem controlSubsystem; ///< used for serial communications
StatusSubsystem statusSubsystem;   ///< provides visual (usually LED) status indicator
struct PhysicalSensor sensors[1];  ///< This implementation uses one physical sensor
EventGroupHandle_t event_group  = NULL;

_Noreturn static void read_task(void *pvParameters);   // FreeRTOS Task definition
_Noreturn static void fusion_task(void *pvParameters); // FreeRTOS Task definition

static const uint8_t CMPS12_i2C_ADDR = 0xC0 >> 1;  //  CMPS12 I2C address
registerDeviceInfo_t i2cBusInfo = {.deviceInstance = 0, .functionParam = NULL, .idleFunction = NULL};

/// This is a FreeRTOS (dual task) implementation of the NXP sensor fusion demo build.
int start_fusing(void)
{
    initializeControlPort(&controlSubsystem);    // configure pins and ports for the control sub-system
    initializeStatusSubsystem(&statusSubsystem); // configure pins and ports for the status sub-system
    initSensorFusionGlobals(&sfg, &statusSubsystem, &controlSubsystem); // Initialize sensor fusion structures
    // "install" the sensors we will be using
    sfg.installSensor(&sfg, &sensors[0], CMPS12_i2C_ADDR, 1, &i2cBusInfo, &i2cBusInfo, CMPS12_Init,
                      CMPS12_Read);
    sfg.initializeFusionEngine(&sfg); // This will initialize sensors and magnetic calibration

    event_group = xEventGroupCreate();
    xTaskCreate(read_task, "READ", 16 * 1024,
                NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(fusion_task, "FUSION", 16 * 1024,
                NULL, tskIDLE_PRIORITY + 1, NULL);
//
//    sfg.setStatus(&sfg, NORMAL);     // If we got this far, let's set status state to NORMAL
    return 0;
}

_Noreturn static void read_task(void *pvParameters)
{
    uint16_t i; // general counter variable
    portTickType lastWakeTime;
    const portTickType xTimeIncrement = 5;  // 50 ms
    lastWakeTime                 = xTaskGetTickCount();
    while (1)
    {
        for (i = 1; i <= OVERSAMPLE_RATE; i++)
        {
            vTaskDelayUntil(&lastWakeTime, xTimeIncrement);
            ESP_LOGD(TAG, "Polling at %lld us", esp_timer_get_time());
            sfg.readSensors(&sfg, i); // Reads sensors, applies HAL and does averaging (if applicable)
        }
        xEventGroupSetBits(event_group, B0);
    }
}

_Noreturn static void fusion_task(void *pvParameters)
{
    uint16_t i = 0; // general counter variable
    while (1)
    {
        xEventGroupWaitBits(event_group,    /* The event group handle. */
                            B0,             /* The bit pattern the event group is waiting for. */
                            pdTRUE,         /* BIT_0 and BIT_4 will be cleared automatically. */
                            pdFALSE,        /* Don't wait for both bits, either bit unblock task. */
                            portMAX_DELAY); /* Block indefinitely to wait for the condition to be met. */

        sfg.conditionSensorReadings(&sfg); // magCal is run as part of this
        sfg.runFusion(&sfg);               // Run the actual fusion algorithms
        sfg.applyPerturbation(&sfg);       // apply debug perturbation (testing only)

        sfg.loopcounter++; // The loop counter is used to "serialize" mag cal operations
        i = i + 1;
        if (i >= 4)
        {                           // Some status codes include a "blink" feature.  This loop
            i = 0;                  // should cycle at least four times for that to operate correctly.
            sfg.updateStatus(&sfg); // This is where pending status updates are made visible
        }
        sfg.queueStatus(&sfg, NORMAL);                          // assume NORMAL status for next pass through the loop
        sfg.pControlSubsystem->stream(&sfg, sUARTOutputBuffer); // Send stream data to the Sensor Fusion Toolbox
    }
}

/// \endcode
