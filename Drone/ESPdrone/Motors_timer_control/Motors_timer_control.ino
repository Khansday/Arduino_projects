#include <Arduino.h>
#include "driver/ledc.h"

const int motorPin1 = 3; // GPIO pin for Motor 1
const int motorPin2 = 4; // GPIO pin for Motor 2
const int motorPin3 = 5; // GPIO pin for Motor 3
const int motorPin4 = 6; // GPIO pin for Motor 4

// Define PWM parameters
const int pwmFreq = 5000;  // 5 kHz PWM frequency
const ledc_timer_bit_t pwmResolution = LEDC_TIMER_13_BIT; // 13-bit resolution (0-8191)
const ledc_timer_t pwmTimer = LEDC_TIMER_0; // Timer 0
const ledc_mode_t pwmSpeedMode = LEDC_LOW_SPEED_MODE;

void setup() {
  // Configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = pwmSpeedMode,
    .duty_resolution  = pwmResolution,
    .timer_num        = pwmTimer,
    .freq_hz          = pwmFreq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Configure LEDC channels
  ledc_channel_config_t ledc_channel1 = {
    .gpio_num       = motorPin1,
    .speed_mode     = pwmSpeedMode,
    .channel        = LEDC_CHANNEL_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = pwmTimer,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel1);

  ledc_channel_config_t ledc_channel2 = {
    .gpio_num       = motorPin2,
    .speed_mode     = pwmSpeedMode,
    .channel        = LEDC_CHANNEL_1,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = pwmTimer,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel2);

  ledc_channel_config_t ledc_channel3 = {
    .gpio_num       = motorPin3,
    .speed_mode     = pwmSpeedMode,
    .channel        = LEDC_CHANNEL_2,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = pwmTimer,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel3);

  ledc_channel_config_t ledc_channel4 = {
    .gpio_num       = motorPin4,
    .speed_mode     = pwmSpeedMode,
    .channel        = LEDC_CHANNEL_3,
    .intr_type      = LEDC_INTR_DISABLE,
    .timer_sel      = pwmTimer,
    .duty           = 0,
    .hpoint         = 0
  };
  ledc_channel_config(&ledc_channel4);
}

void loop() {
  // Set duty cycle to 50% (half speed) for all motors
  int dutyCycle = 4096; // 50% of 13-bit range (8191)
  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_0, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_0);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_1, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_1);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_2, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_2);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_3, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_3);
  
  delay(1000); // Run motors at half speed for 1 second
  
  // Set duty cycle to 100% (full speed) for all motors
  dutyCycle = 8191; // Full resolution for 13-bit
  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_0, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_0);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_1, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_1);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_2, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_2);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_3, dutyCycle);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_3);
  
  delay(1000); // Run motors at full speed for 1 second
  
  // Stop all motors
  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_0, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_0);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_1, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_1);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_2, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_2);

  ledc_set_duty(pwmSpeedMode, LEDC_CHANNEL_3, 0);
  ledc_update_duty(pwmSpeedMode, LEDC_CHANNEL_3);
  
  delay(1000); // Motors off for 1 second
}