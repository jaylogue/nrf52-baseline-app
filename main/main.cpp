/**
 * Copyright (c) 2020 Jay Logue
 * All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 */

/**
 *   @file
 *         main function for the nrf52-baseline-app embedded application.
 */

#include <stdbool.h>
#include <stdint.h>
// #include <malloc.h>
#include <inttypes.h>

#include <sdk_common.h>
#include <boards.h>
#include <app_timer.h>
#include <mem_manager.h>
#include <nrf_pwr_mgmt.h>
#include <nrf_drv_clock.h>
#include <nrf_delay.h>
#include <app_button.h>

#if NRF_LOG_ENABLED
#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>
#endif // NRF_LOG_ENABLED

#if NRF_CRYPTO_ENABLED
#include <nrf_crypto.h>
#endif

#include <SimpleBLEApp.h>
#include <LEDButtonService.h>
#include <BLEEventLogger.h>
#include <nRF5SysTime.h>
#include <nRF5Utils.h>

using namespace nrf5utils;

APP_TIMER_DEF(sStatusLEDTimer);

static SIMPLE_EVENT_OBSERVER(sOnAdvertisingStarted, SimpleBLEApp::Event::OnAdvertisingStarted, 0,
    []() {
        app_timer_stop(sStatusLEDTimer);
        app_timer_start(sStatusLEDTimer, APP_TIMER_TICKS(APP_STATUS_LED_BLINK_INTERVAL), NULL);
    }
);

static SIMPLE_EVENT_OBSERVER(sOnConnectionEstablished, SimpleBLEApp::Event::OnConnectionEstablished, 0,
    [](uint16_t conHandle, const ble_gap_evt_connected_t * conEvent) {
        app_timer_stop(sStatusLEDTimer);
        nrf_gpio_pin_clear(APP_STATUS_LED_PIN);
    }
);

static SIMPLE_EVENT_OBSERVER(sOnConnectionTerminated, SimpleBLEApp::Event::OnConnectionTerminated, 0,
    [](uint16_t conHandle, const ble_gap_evt_disconnected_t * disconEvent) {
        app_timer_stop(sStatusLEDTimer);
        nrf_gpio_pin_set(APP_STATUS_LED_PIN);
    }
);

static SIMPLE_EVENT_OBSERVER(sOnLEDWrite, LEDButtonService::Event::OnLEDWrite, 0,
    [](bool setOn) {
        nrf_gpio_pin_write(APP_UI_LED_PIN, setOn ? 0 : 1);
    }
);

static void StatusLEDTimerHandler(void * context)
{
    nrf_gpio_pin_toggle(APP_STATUS_LED_PIN);
}

int main(void)
{
    ret_code_t res;

#if JLINK_MMD
    NVIC_SetPriority(DebugMonitor_IRQn, _PRIO_APP_LOW);
#endif

    // Initialize clock driver.
    res = nrf_drv_clock_init();
    APP_ERROR_CHECK(res);

    // Start the low-frequency clock and wait for it to be ready.
    nrf_drv_clock_lfclk_request(NULL);
    while (!nrf_clock_lf_is_running()) { }

#if NRF_LOG_ENABLED

    // Initialize logging component
#if NRF_LOG_USES_TIMESTAMP
    res = NRF_LOG_INIT(SysTime::GetSystemTime_MS32, 1000);
#else
    res = NRF_LOG_INIT(NULL, 0);
#endif
    APP_ERROR_CHECK(res);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

#endif // NRF_LOG_ENABLED

    NRF_LOG_INFO("==================================================");
    NRF_LOG_INFO("nrf52-baseline-app starting");
    NRF_LOG_INFO("==================================================");

    // Initialize the app_timer module.
    res = app_timer_init();
    NRF_LOG_CALL_FAIL_INFO("app_timer_init", res);
    APP_ERROR_CHECK(res);

    // Initialize the SysTime module.
    res = SysTime::Init();
    NRF_LOG_CALL_FAIL_INFO("SysTime::Init", res);
    APP_ERROR_CHECK(res);

    // Initialize the power management module.
    res = nrf_pwr_mgmt_init();
    NRF_LOG_CALL_FAIL_INFO("nrf_pwr_mgmt_init", res);
    APP_ERROR_CHECK(res);

    // Initialize the nRF5 SDK Memory Manager
    res = nrf_mem_init();
    NRF_LOG_CALL_FAIL_INFO("nrf_mem_init", res);
    APP_ERROR_CHECK(res);

#if NRF_CRYPTO_ENABLED

    // Initialize the nrf_crypto library.
    res = nrf_crypto_init();
    NRF_LOG_CALL_FAIL_INFO("nrf_crypto_init", res);
    APP_ERROR_CHECK(res);

#endif // NRF_CRYPTO_ENABLED

    res = SimpleBLEApp::Init();
    NRF_LOG_CALL_FAIL_INFO("SimpleBLEApp::Init", res);
    APP_ERROR_CHECK(res);

    res = BLEEventLogger::Init();
    NRF_LOG_CALL_FAIL_INFO("BLEEventLogger::Init", res);
    APP_ERROR_CHECK(res);

    res = LEDButtonService::Init();
    NRF_LOG_CALL_FAIL_INFO("LEDButtonService::Init", res);
    APP_ERROR_CHECK(res);

    // Create and start a timer to toggle the status LED
    res = app_timer_create(&sStatusLEDTimer, APP_TIMER_MODE_REPEATED, StatusLEDTimerHandler);
    NRF_LOG_CALL_FAIL_INFO("app_timer_create", res);
    APP_ERROR_CHECK(res);

    // Initialize status and UI LED GPIOs
    nrf_gpio_cfg_output(APP_STATUS_LED_PIN);
    nrf_gpio_pin_set(APP_STATUS_LED_PIN);
    nrf_gpio_cfg_output(APP_UI_LED_PIN);
    nrf_gpio_pin_set(APP_UI_LED_PIN);

    // Initialize the UI button GPIO and the app_button library.
    nrf_gpio_cfg_input(APP_UI_BUTTON_PIN, APP_UI_BUTTON_PULL_CONFIG);
    static app_button_cfg_t sButtonConfigs[] =
    {
        { APP_UI_BUTTON_PIN, APP_UI_BUTTON_ACTIVE_STATE, APP_UI_BUTTON_PULL_CONFIG, LEDButtonService::ButtonEventHandler }
    };
    res = app_button_init(sButtonConfigs, ARRAY_SIZE(sButtonConfigs), APP_BUTTON_DETECTION_DELAY);
    NRF_LOG_CALL_FAIL_INFO("app_button_init", res);
    APP_ERROR_CHECK(res);
    res = app_button_enable();
    NRF_LOG_CALL_FAIL_INFO("app_button_enable", res);
    APP_ERROR_CHECK(res);

    NRF_LOG_INFO("System initialization complete");

#if NRF_LOG_ENABLED && NRF_LOG_LEVEL >= NRF_LOG_SEVERITY_INFO
    LogHeapStats();
#endif

    res = SimpleBLEApp::StartAdvertising();
    NRF_LOG_CALL_FAIL_INFO("SimpleBLEApp::StartAdvertising", res);
    APP_ERROR_CHECK(res);

    NRF_LOG_INFO("Starting main loop");

    while (true)
    {
#if NRF_LOG_DEFERRED
        while (NRF_LOG_PROCESS())
            ;
#endif

        res = SimpleBLEApp::RunMainLoopActions();
        NRF_LOG_CALL_FAIL_INFO("SimpleBLEApp::RunMainLoopActions", res);
        APP_ERROR_CHECK(res);

        nrf_pwr_mgmt_run();
    }
}
