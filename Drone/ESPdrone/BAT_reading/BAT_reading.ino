#include "driver/adc.h"
#include "esp_adc_cal.h"

#define ADC_PIN ADC1_CHANNEL_1  // GPIO2 corresponds to ADC1_CHANNEL_1
#define DEFAULT_VREF 1100        // Default Vref is 1100 mV

esp_adc_cal_characteristics_t *adc_chars;


void setup() {
  Serial.begin(921600);
  // Configure the ADC
  adc1_config_width(ADC_WIDTH_BIT_13);  // 13-bit width
  adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_11);  //max attenuation

  // Calibrate the ADC
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_13, DEFAULT_VREF, adc_chars);
}

void loop() {
  // Read the raw ADC value
  int rawValue = adc1_get_raw(ADC_PIN);  // Use ADC1_CHANNEL_1 for GPIO 2

  // Convert the raw value to voltage
  uint32_t voltage = (esp_adc_cal_raw_to_voltage(rawValue, adc_chars))*2;

  // Print the raw value and the calculated voltage
  Serial.print("Raw ADC Value: ");
  Serial.print(rawValue);
  Serial.print(" | Calculated Voltage: ");
  Serial.print(voltage);
  Serial.println(" mV");

  delay(1000);  // Delay 1 second before next reading
}
