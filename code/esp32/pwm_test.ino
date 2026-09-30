// PWM test on an ESP32: 1 kHz, 8-bit resolution, alternating 25% and 75% duty.
// Captured on a sigrok FX2 logic analyzer in PulseView.
int pwmPin = 18;
int freq = 1000;        // 1 kHz
int resolution = 8;     // 8-bit: duty 0-255

void setup() {
  ledcAttach(pwmPin, freq, resolution);
}

void loop() {
  ledcWrite(pwmPin, 64);    // ~25% duty
  delay(300);
  ledcWrite(pwmPin, 191);   // ~75% duty
  delay(300);
}
