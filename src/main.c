/*
 * Copyright (c) 2024 Marko Vejnovic
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @detail
 * Hello! This module is the main entrypoint of the biologger firmware. It is
 * in this file that you should most likely attempt to perform your work. If
 * you are only attempting to add new columns/rows, please have a look at
 * declare_columns and collect_data_10hz.
 */
#include <sys/_timespec.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include "experiment.h"
#include "observer.h"
#include <zephyr/device.h>
#include "trutime.h"
#include "storage.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include "sensor/ximpedance_amp/ximpedance_amp.h"
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/sensor.h>

#define SAMPLING_PERIOD_MS 100

LOG_MODULE_REGISTER(main);

OBSERVER_DECL(main_observer);
TRUTIME_DECL(time_provider);

/******************************************************************************
 * Devices Used On The Board
 *****************************************************************************/
static const struct device* ximpedance_amp =
    DEVICE_DT_GET(DT_NODELABEL(ximpedance_amp));

/******************************************************************************
 * Static Variables Used In This Module
 *****************************************************************************/
static size_t collection_counter = 0;

/**
 * @brief Initialize drivers required for the operation of the application.
 */
static int init_drivers() {
    int err = 0; // 0 means no error :)

    if (!device_is_ready(ximpedance_amp)) {
        LOG_ERR("Failed to initialize the transimpedance driver.");
        err = -ENODEV;
    }

    return err;
}

/**
 * @brief Define all the columns that are collected.
 *
 * This function must only contain calls to experiment_add_column.
 */
static void declare_columns(struct experiment* e) {
    //                           Column Name              Units
    experiment_add_column(e,     "Current 22KX 1",        "mA");
    experiment_add_column(e,     "Current 22KX 2",        "mA");
    experiment_add_column(e,     "Current 10KX 1",        "mA");
    experiment_add_column(e,     "Current 10KX 2",        "mA");
}

/**
 * @brief Perform all data collection.
 *
 * This function must only contain calls to experiment_row_add_value.
 *
 * @warning There must be as many calls to experiment_row_add_value as there
 *          are to experiment_add_column in declare_columns.
 */
static int collect_data_10hz(struct experiment_row* r) {
    int err = 0; // 0 means no error :)
    
    if ((err = sensor_sample_fetch(ximpedance_amp)) != 0) {
        LOG_ERR("Failed to sample the results from the transimpedance "
                "amplifier (%d).", err);
    }

    static const enum ximpedance_amp_sensor_channel
    ximpedance_amp_channels[] = {
        XIMPEDANCE_CHAN_22KX_MILLIAMPS_1,
        XIMPEDANCE_CHAN_22KX_MILLIAMPS_2,
        XIMPEDANCE_CHAN_10KX_MILLIAMPS_1,
        XIMPEDANCE_CHAN_10KX_MILLIAMPS_2,
    };
    for (size_t i = 0; i < ARRAY_SIZE(ximpedance_amp_channels); i++) {
        struct sensor_value val;
        if ((err = sensor_channel_get(ximpedance_amp,
                                      (int)ximpedance_amp_channels[i],
                                      &val)) != 0) {
            LOG_ERR("Failed to fetch the sensor channel %d value (%d).",
                    ximpedance_amp_channels[i], err);
            err = MIN(err, 0);
        }

        const double value_milliamps = sensor_value_to_double(&val);

        experiment_row_add_value(r, value_milliamps);
    }

    // Printout every 10th row.
    if (collection_counter == 0) {
        const struct strv printout = experiment_row_format(r);
        printk("%s\n", printout.str);
        k_free(printout.str);
    }
    collection_counter = (collection_counter + 1) % 10;

    return err;
}

int main(void) {
    int err;

    // Initialize the logging module which is invaluable when debugging.
    LOG_INIT();
    LOG_INF("Biologger 15");
    LOG_INF("Initializing modules...");

    // Initialize the observer which will oversee all the operations and blink
    // the LED.
    observer_t observer = OBSERVER_INIT(main_observer);

    // Initialize the storage module which is responsible for storing
    // experiment data.
    storage_t storage = storage_init(observer);
    if (storage == NULL) {
        LOG_ERR("Could not initialize storage.");
        return -ENOMEM;
    }

    // Initialize the trutime module which provides with accurate time data.
    trutime_t time_provider = TRUTIME_INIT(time_provider, observer);

    // Initialize the hardware drivers.
    if ((err = init_drivers()) != 0) {
        LOG_ERR("Failed to initialize a driver (%d).", err);
        observer_flag_raise(observer, OBSERVER_FLAG_DRIVER_MIA);
    }

    // Wait until trutime is available. Sometimes this takes quite some time.
    do {
        LOG_INF("Waiting for trutime support.");
        k_msleep(1000);
    } while (!trutime_is_available(time_provider));
    k_msleep(1000); // TODO(markovejnovic): trutime_is_available leaks before
                    // it is actually available.

    // Initialize the experiment 
    struct experiment* experiment = experiment_init(storage, time_provider);
    if (experiment == NULL) { 
        LOG_ERR("Failed to initialize the experiment");
        return -1;
    }

    // Populate the experiment with all the required columns.
    declare_columns(experiment);

    while (1) {
        const uint64_t start = k_uptime_get();

        // We are creating a new time sample -- create a new row.
        struct experiment_row* row = experiment_row_new(
            trutime_millis_since(
                time_provider,
                experiment_start_time(experiment)
            )
        );
        if (row == NULL) {
            LOG_ERR("Failed to allocate sufficient memory for a new row.");
            continue;
        }

        // Collect the specified data into the experiment.
        (void)collect_data_10hz(row);

        // Push these values into the experiment.
        if ((err = experiment_push_row(experiment, row)) != 0) {
            LOG_ERR("Failed to push a row into the experiment (%d)", err);
        }

        const uint64_t stop = k_uptime_get();
        int64_t sleep_period;
        const bool sleep_will_underflow = __builtin_sub_overflow(
            (int64_t)stop,
            (int64_t)start,
            &sleep_period
        );
        if (sleep_will_underflow) {
            LOG_ERR("Critically slow application causing sampling lag.");
        } else {
            k_msleep(SAMPLING_PERIOD_MS - (sleep_period));
        }
    }
}
