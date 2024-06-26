/*
 *
 *    Copyright (c) 2022-2023 Project CHIP Authors
 *    All rights reserved.
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
 */

#include "AppTask.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include "AppEvent.h"

#define APP_TASK_NAME "APP"
#define APP_EVENT_QUEUE_SIZE 10
#define APP_TASK_STACK_SIZE (3072)

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceLayer;

static const char* TAG = "app-task";

namespace {
    constexpr EndpointId kTempEndpointId = 1;
    QueueHandle_t sAppEventQueue;
    TaskHandle_t sAppTaskHandle;
    TimerHandle_t sTimerHandle;
} // namespace

AppTask AppTask::sAppTask;

CHIP_ERROR AppTask::StartAppTask() {
    sAppEventQueue = xQueueCreate(APP_EVENT_QUEUE_SIZE, sizeof(AppEvent));
    if (sAppEventQueue == NULL) {
        ESP_LOGE(TAG, "Failed to allocate app event queue");
        return APP_ERROR_EVENT_QUEUE_FAILED;
    }

/*
    sTimerHandle = xTimerCreate(
        "Timer",
        pdMS_TO_TICKS(60000),
        pdTRUE,
        (void*)0,
        TimerTimeoutHandler
    );
    if (sTimerHandle != NULL) {
        xTimerStart(sTimerHandle, 0);
    } else {
        ESP_LOGE(TAG, "Unable to create timer");
    }
*/

    // Start App task.
    BaseType_t xReturned;
    xReturned = xTaskCreate(AppTaskMain, APP_TASK_NAME, APP_TASK_STACK_SIZE, NULL, 1, &sAppTaskHandle);
    return (xReturned == pdPASS) ? CHIP_NO_ERROR : APP_ERROR_CREATE_TASK_FAILED;
}

/*void AppTask::TimerTimeoutHandler(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Timer timeout");
    float temp = temperature.readTemperature();
    ESP_LOGI(TAG, "Temp: %f", temp);

    AppEvent timeout_event = {};
    timeout_event.Type = AppEvent::kEventType_Temp;
    timeout_event.TemperatureEvent.Temp = temp;
    timeout_event.mHandler = TemperatureActionEventHandler;
    sAppTask.PostEvent(&timeout_event);
}*/

bool inHandleButton = false;
uint64_t lastIRSTime = 0;
void AppTask::gpio_isr_contact_sensor_handler(void* arg) {
    if((esp_timer_get_time() - lastIRSTime) > 1000000) {
        inHandleButton = false;
    }
    lastIRSTime = esp_timer_get_time();
    if(inHandleButton) {
        return;
    }
    inHandleButton = true;

    //time_door_event = esp_timer_get_time();
    //esp_event_post_to(loop_handle, TEEL_DOOR_EVENTS, TEEL_DOOR_EVENT, 
    //    &time_door_event, sizeof(time_door_event), portMAX_DELAY);
    AppEvent aEvent = {};
    aEvent.Type = AppEvent::kEventType_OpenClose;
    aEvent.mHandler = NULL;
    sAppTask.PostEvent(&aEvent);
    
}

CHIP_ERROR AppTask::Init() {
    ESP_LOGI(TAG, "--> Setup door sensor <--");
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << 11,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)11, 
        gpio_isr_contact_sensor_handler, (void*) 11));
    
    CHIP_ERROR err = CHIP_NO_ERROR;
//    chip::DeviceLayer::PlatformMgr().LockChipStack();
//    app::Clusters::TemperatureMeasurement::Attributes::MinMeasuredValue::Set(kTempEndpointId, 0);
//    app::Clusters::TemperatureMeasurement::Attributes::MaxMeasuredValue::Set(kTempEndpointId, 50);
//    chip::DeviceLayer::PlatformMgr().UnlockChipStack();
    return err;
}

void AppTask::AppTaskMain(void* pvParameter) {
    AppEvent event;
    CHIP_ERROR err = sAppTask.Init();
    if (err != CHIP_NO_ERROR) {
        ESP_LOGI(TAG, "AppTask.Init() failed due to %" CHIP_ERROR_FORMAT, err.Format());
        return;
    }

    ESP_LOGI(TAG, "App Task started");

    while (true) {
        BaseType_t eventReceived = xQueueReceive(sAppEventQueue, &event, pdMS_TO_TICKS(10));
        while (eventReceived == pdTRUE) {
            sAppTask.DispatchEvent(&event);
            eventReceived = xQueueReceive(sAppEventQueue, &event, 0); // return immediately if the queue is empty
        }
    }
}

void AppTask::PostEvent(const AppEvent* aEvent) {
    if (sAppEventQueue != NULL) {
        BaseType_t status;
        if (xPortInIsrContext()) {
            BaseType_t higherPrioTaskWoken = pdFALSE;
            status = xQueueSendFromISR(sAppEventQueue, aEvent, &higherPrioTaskWoken);
        } else {
            status = xQueueSend(sAppEventQueue, aEvent, 1);
        }
        if (!status)
            ESP_LOGE(TAG, "Failed to post event to app task event queue");
    } else {
        ESP_LOGE(TAG, "Event Queue is NULL should never happen");
    }
}

void AppTask::DispatchEvent(AppEvent* aEvent) {
    if (aEvent->mHandler) {
        aEvent->mHandler(aEvent);
    } else { 
        if(aEvent->Type == AppEvent::kEventType_OpenClose) {
            ESP_LOGI(TAG, "OpenClose event received");

            chip::DeviceLayer::PlatformMgr().LockChipStack();

            int pin_level = gpio_get_level((gpio_num_t)11);

            EmberAfStatus status = app::Clusters::BooleanState::Attributes::StateValue::Set(
                kTempEndpointId, pin_level);
            if (status != EMBER_ZCL_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Updating temperature cluster failed: %x", status);
            }

            chip::DeviceLayer::PlatformMgr().UnlockChipStack();

        } else {
            ESP_LOGI(TAG, "Unknown event!");
        }
        
    }
}

/*void AppTask::TemperatureActionEventHandler(AppEvent* aEvent) {
    //chip::DeviceLayer::PlatformMgr().LockChipStack();
    //sAppTask.UpdateClusterState();
    //chip::DeviceLayer::PlatformMgr().UnlockChipStack();
    chip::DeviceLayer::PlatformMgr().LockChipStack();
    EmberAfStatus status = app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
        kTempEndpointId, (int16_t)aEvent->TemperatureEvent.Temp * 100);
    if (status != EMBER_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Updating temperature cluster failed: %x", status);
    }
    chip::DeviceLayer::PlatformMgr().UnlockChipStack();

}
*/

void AppTask::UpdateClusterState() {
    ESP_LOGI(TAG, "Writing to OnOff cluster");
    // write the new on/off value
    EmberAfStatus status = EMBER_ZCL_STATUS_SUCCESS; //Clusters::OnOff::Attributes::OnOff::Set(kLightEndpointId, AppLED.IsTurnedOn());

    if (status != EMBER_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Updating on/off cluster failed: %x", status);
    }

    ESP_LOGI(TAG, "Writing to Current Level cluster");
    status = EMBER_ZCL_STATUS_SUCCESS; //Clusters::LevelControl::Attributes::CurrentLevel::Set(kLightEndpointId, AppLED.GetLevel());

    if (status != EMBER_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Updating level cluster failed: %x", status);
    }
}
