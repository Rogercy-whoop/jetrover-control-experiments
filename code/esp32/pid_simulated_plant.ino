// PID on a simulated plant, running on an ESP32 (Arduino IDE 2.3.10)
// Final version: Kp/Ki/Kd baseline + integral clamp (anti-windup).
// The "plant" is a simple model with inertia and damping, not a real motor.

float target = 100.0;
float current = 0.0;
float velocity = 0.0;          // plant state: velocity / inertia
float Kp = 0.8, Ki = 0.02, Kd = 0.3;
float integral = 0, lastError = 0;
float dt = 0.05;               // must match the loop delay below (50 ms)
int step = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  float error = target - current;
  integral += error * dt;
  if (integral > 50) integral = 50;      // anti-windup: clamp the integral
  if (integral < -50) integral = -50;
  float derivative = (error - lastError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Simulated physics: output acts like a force on velocity,
  // velocity changes position, and damping lets the system stop.
  velocity += output * dt;
  velocity *= 0.9;                        // damping / friction
  current += velocity * dt;

  Serial.print(target);
  Serial.print(",");
  Serial.println(current);

  lastError = error;
  step++;
  if (step > 1500) { while (true) { delay(1000); } }
  delay(50);
}
