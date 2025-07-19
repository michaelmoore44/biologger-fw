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
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "stm32_ll_rcc.h"
#include <zephyr/drivers/clock_control.h>

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
    LOG_INF(" ");
    LOG_INF(" ");
    LOG_INF("Biologger 13k");
    LOG_INF(" ");
    LOG_INF(" ");
    LOG_INF("Initializing modules...");

    // Initialize the observer which will oversee all the operations and blink
    // the LED.
    observer_t observer = OBSERVER_INIT(main_observer);


    #if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(sdmmc1))
    #warning "Missing sdmmc1"
    #endif

    #if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_sdmmc)
    #define DT_DRV_COMPAT st_stm32_sdmmc1
    #endif

    #if !DT_HAS_COMPAT_STATUS_OKAY(st_stm32_clock_mux)
    #warning "Missing clock 48MHz"
    #endif

    #ifndef STM32_SRC_CK48
    #error "Missing clock CK48"
    #endif

    #ifndef STM32_CK48_ENABLED
    #warning "Missing clock CK48_ENABLED"
    #endif


	static const struct stm32_pclken pclken[] = STM32_DT_CLOCKS(DT_NODELABEL(sdmmc1));

	uint32_t dev_dt_clk_freq, dev_actual_clk_freq, print_val;
	uint32_t dev_actual_clk_src;
	int r;


	/* Test clock_on(gating clock) */
	r = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t) &pclken[0]);
	if(!(r == 0))
        LOG_ERR("Could not enable SDMMC gating clock");

	if(!(__HAL_RCC_SDIO_IS_CLK_ENABLED()))
        LOG_ERR("SDMMC gating clock should be on");

	LOG_INF("SDMMC gating clock on");

	if(!(DT_NUM_CLOCKS(DT_NODELABEL(sdmmc1)) > 1))
        LOG_ERR("No domain clock defined in dts");

	if (pclken[1].bus == STM32_SRC_CK48) {
		/* CLK 48 is enabled through the clock-mux */
		if(!(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(clk48))))
            LOG_ERR("No clock 48MHz");
		r = 0;
	} else if (pclken[1].bus == STM32_SRC_SYSCLK) {
		/* Test clock_on(domain_clk) STM32_SRC_SYSCLK */
		r = clock_control_configure(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					(clock_control_subsys_t) &pclken[1],
					NULL);
	} else {
		r = -127;
	}

	if(!(r == 0))
        LOG_ERR("Could not enable SDMMC domain clock");
	else 
    {
        LOG_INF("SDMMC domain clock configured");
        LOG_INF(" ");
    }





	/* Test clock source */
	dev_actual_clk_src = __HAL_RCC_GET_SDIO_SOURCE();

	if (pclken[1].bus == STM32_SRC_CK48)
    {
        LOG_INF("STM32_SRC_CK48");
		if(!(dev_actual_clk_src == RCC_SDIOCLKSOURCE_CLK48))
            LOG_ERR("Expected SDMMC src: CLK 48 (0x%x). Actual src: 0x%x", RCC_SDIOCLKSOURCE_CLK48, dev_actual_clk_src);
        else
            LOG_INF("SDMMC clock is RCC_SDIOCLKSOURCE_CLK48");
	}
    else if (pclken[1].bus == STM32_SRC_SYSCLK)
    {
        LOG_INF("STM32_SRC_SYSCLK\n");
		if(!(dev_actual_clk_src == RCC_SDIOCLKSOURCE_SYSCLK))
            LOG_ERR("Expected SDMMC src: SYSCLK (0x%x). Actual src: 0x%x", RCC_SDIOCLKSOURCE_SYSCLK, dev_actual_clk_src);
        else
            LOG_INF("SDMMC clock is RCC_SDIOCLKSOURCE_SYSCLK");
	}
    else
    {
		LOG_ERR("Unexpected domain clk (0x%x)", dev_actual_clk_src);
	}

	/* Test get_rate(srce clk) */
	if (pclken[1].bus == STM32_SRC_CK48) {
		/* Get the CK48M source : PLL Q or PLLI2S Q */
		if (LL_RCC_GetCK48MClockSource(LL_RCC_CK48M_CLKSOURCE) ==
				LL_RCC_CK48M_CLKSOURCE_PLL) {
			/* Get the PLL Q freq. No HAL macro for that */
			dev_actual_clk_freq = __LL_RCC_CALC_PLLCLK_48M_FREQ(HSE_VALUE,
							    LL_RCC_PLL_GetDivider(),
							    LL_RCC_PLL_GetN(),
							    LL_RCC_PLL_GetQ()
							    );
			LOG_INF("SDMMC sourced by PLLQ at %d Hz", dev_actual_clk_freq);
		} else {
			/* Get the I2S PLL Q freq. No HAL macro for that */
			dev_actual_clk_freq = __LL_RCC_CALC_PLLI2S_48M_FREQ(HSE_VALUE,
							    LL_RCC_PLLI2S_GetDivider(),
							    LL_RCC_PLLI2S_GetN(),
							    LL_RCC_PLLI2S_GetQ()
							    );
			LOG_INF("SDMMC sourced by PLLI2SQ at %d Hz", dev_actual_clk_freq);
		}

        print_val = LL_RCC_PLL_GetDivider();
		LOG_INF("Divider: %d", print_val);
        print_val = LL_RCC_PLL_GetN();
		LOG_INF("N: %d", print_val);
        print_val = LL_RCC_PLL_GetQ();
		LOG_INF("Q: %d", print_val);
		r = 0;

	}
    else if (pclken[1].bus == STM32_SRC_SYSCLK)
    {
		dev_actual_clk_freq = HAL_RCC_GetSysClockFreq();
		LOG_INF(" STM32_SRC_SYSCLK at %d\n", dev_actual_clk_freq);
	}
    else
    {
		r = -127;
	}

	if(!(r == 0))
        LOG_INF("Could not get SDMMC clk srce freq");
    else
        LOG_INF("Got SDMMC clk srce freq");


	r = clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t) &pclken[1],
				&dev_dt_clk_freq);

	if(!(dev_dt_clk_freq == dev_actual_clk_freq))
        LOG_INF("Expected freq: %d Hz. Actual clk: %d Hz", dev_dt_clk_freq, dev_actual_clk_freq);

	LOG_INF("SDMMC clock rate: %d Hz\n", dev_dt_clk_freq);

	/* Test clock_off(gating clk) */
    /*
	r = clock_control_off(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t) &pclken[0]);
	if(!(r == 0))
        LOG_ERR("Could not disable SDMMC gating clk");

	if(__HAL_RCC_SDIO_IS_CLK_ENABLED())
        LOG_ERR("SDMMC gating clk should be off");

	LOG_INF("SDMMC gating clk off\n");
*/



	if(__HAL_RCC_SDIO_IS_CLK_ENABLED())
        LOG_INF("SDMMC CLK is enabled");
    else
    {
        LOG_ERR("SDMMC CLK is not enabled");
        LOG_ERR("");
    }



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
