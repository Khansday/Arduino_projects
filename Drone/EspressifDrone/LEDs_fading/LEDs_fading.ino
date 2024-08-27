#include "soc/gpio_struct.h"
#include "driver/ledc.h"
#include "esp32-hal-ledc.h"

#define LED_BLE_PIN 7
#define LED_RED_PIN 8
#define LED_GRN_PIN 9

void setupGPIO() {
    // Set GPIOs as outputs using direct port manipulation
    GPIO.enable_w1ts = (1 << LED_BLE_PIN) | (1 << LED_RED_PIN) | (1 << LED_GRN_PIN);
}

void setupPWM() {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channels
    ledc_channel_config_t ledc_channel0 = {
        .gpio_num = LED_BLE_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel0);

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = LED_RED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = LED_GRN_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel2);
}

void setPWMDuty(uint8_t channel, uint8_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

void delayMs(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        // Busy-wait loop for delay
    }
}

void setup() {
    setupGPIO();
    setupPWM();
}

void loop() {
    // Fade in from 0 to 255
    for (int brightness = 0; brightness <= 255; brightness++) {
        setPWMDuty(LEDC_CHANNEL_0, brightness);
        setPWMDuty(LEDC_CHANNEL_1, brightness);
        setPWMDuty(LEDC_CHANNEL_2, brightness);
        delayMs(10);  // Adjust the delay to control the fade speed
    }

    // Fade out from 255 to 0
    for (int brightness = 255; brightness >= 0; brightness--) {
        setPWMDuty(LEDC_CHANNEL_0, brightness);
        setPWMDuty(LEDC_CHANNEL_1, brightness);
        setPWMDuty(LEDC_CHANNEL_2, brightness);
        delayMs(10);  // Adjust the delay to control the fade speed
    }
}

