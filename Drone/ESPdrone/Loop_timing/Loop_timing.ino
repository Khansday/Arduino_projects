const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
const int CONFIG = 0x1A;  // Address of the low pass filter register
float alpha = 0.98;  // Initial complementary filter constant

// Add these variables for offsets
int16_t ax_offset = -263, ay_offset = -95, az_offset = -1157;
int16_t gx_offset = -226, gy_offset = 58, gz_offset = 179;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
double dt = 0.001; // Initial dt value for the first loop
int loopFreq = 1000;  //in microseconds


int i=0, cyclesPrint = 10;  //debug printing variable and cycles

void setup() {
  Serial.begin(921600); // highest speed possible on my setup, higher than this I get errors
  // Initial delay to allow the sensor to stabilize
  delay(100);
}

void loop() {
  currentTime = micros();
  dt = (currentTime - lastTime); // Convert dt to seconds

  if (dt >= loopFreq) { // Ensure the loop runs at approximately 1 kHz (1ms per loop)
    lastTime = currentTime;  
    i++;
    // Print the results
    if (i = cyclesPrint){
      debugPrint();
      i = 0;
    }
  }
}



void debugPrint(){
    // Print the results
    Serial.print("T: ");
    Serial.print(dt);
    Serial.print(" us ");
    Serial.print(dt/1000);
    Serial.print(" ms ");
    Serial.print(dt/1000000,4);
    Serial.print(" s ");
    Serial.print("F: ");
    Serial.println(1000000/dt);
}

