const int pulsePin = A0;  // Pin where the pulse sensor is connected
unsigned long sampleWindow;  // Sample window width in milliseconds
int pulse_signal;  // Variable to store the analog value from the pulse sensor
unsigned long sampleStartTime;
unsigned long lastBeatTime;
int beatCount;
float BPM;
int threshold = 590;  // Threshold for detecting a beat
const int samplingRate = 10;  // Sampling rate in seconds
bool LED_state = true;

void setup() {
  Serial.begin(460800);
  pinMode(LED_BUILTIN,OUTPUT);  // Built-in LED will blink to your heartbeat
  pinMode(pulsePin, INPUT);  // Set the pulse sensor pin as input
  sampleWindow = samplingRate * 1000;  // Convert sampling rate to milliseconds
  sampleStartTime = millis();  // Initialize the sample start time
  lastBeatTime = 0;  // Initialize the last beat time
  beatCount = 0;  // Initialize the beat count
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read the analog value from the pulse sensor
  pulse_signal = analogRead(pulsePin);
  
  // Check if the pulse_signal exceeds the threshold, indicating a beat
  if (pulse_signal > threshold) {
    if ((currentTime - lastBeatTime) > 150) {  // Debounce the pulse_signal
      beatCount++;
      lastBeatTime = currentTime;
      LED_state = digitalRead(LED_BUILTIN);
      digitalWrite(LED_BUILTIN,!LED_state);
    }
  }
  
  // Calculate BPM every sample window
  if (currentTime - sampleStartTime >= sampleWindow) {
    // BPM = float(beatCount / samplingRate) * (60 / samplingRate) ;
    // Serial.print("BPM: ");
    Serial.println(BPM);
    
    // Reset the sample window
    sampleStartTime = currentTime;
    beatCount = 0;
  }
}
