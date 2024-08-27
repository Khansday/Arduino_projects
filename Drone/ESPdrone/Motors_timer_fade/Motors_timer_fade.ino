#include <Arduino.h>
#include "driver/ledc.h"

// Define GPIO pins connected to motor drivers
const int motorPin1 = 3;
const int motorPin2 = 4;
const int motorPin3 = 5;
const int motorPin4 = 6;

// Define PWM parameters
const int pwmFreq = 5000;  // 5 kHz PWM frequency
const ledc_timer_bit_t pwmResolution = LEDC_TIMER_13_BIT; // 13-bit resolution (0-8191)
const ledc_timer_t pwmTimer = LEDC_TIMER_0; // Timer 0
const ledc_mode_t pwmSpeedMode = LEDC_LOW_SPEED_MODE;

void setup() {
  // Install LEDC fade service
  ledc_fade_func_install(0);

  // Configure the LEDC timer
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = pwmSpeedMode,
    .duty_resolution  = pwmResolution,
    .timer_num        = pwmTimer,
    .freq_hz          = pwmFreq,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Configure LEDC channels for each motor
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
  // Fade motors from 0 to maximum
  int maxDutyCycle = 8191; // Max for 13-bit resolution
  int fadeTime = 3000; // Time in milliseconds

  // Set fade configurations
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_0, maxDutyCycle, fadeTime);
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_1, maxDutyCycle, fadeTime);
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_2, maxDutyCycle, fadeTime);
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_3, maxDutyCycle, fadeTime);

  // Start fades simultaneously
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_2, LEDC_FADE_NO_WAIT);
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_3, LEDC_FADE_NO_WAIT);

  delay(fadeTime + 1000); // Wait for fade to complete plus 1 second delay

  // Fade motors from maximum to 0
  int minDutyCycle = 0; // Minimum duty cycle

  // Set fade configurations
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_0, minDutyCycle, fadeTime);
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_1, minDutyCycle, fadeTime);
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_2, minDutyCycle, fadeTime);
  ledc_set_fade_with_time(pwmSpeedMode, LEDC_CHANNEL_3, minDutyCycle, fadeTime);

  // Start fades simultaneously
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_2, LEDC_FADE_NO_WAIT);
  ledc_fade_start(pwmSpeedMode, LEDC_CHANNEL_3, LEDC_FADE_NO_WAIT);

  delay(fadeTime + 1000); // Wait for fade to complete plus 1 second delay
}
