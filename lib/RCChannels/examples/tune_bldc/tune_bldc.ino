#include <Servo.h>

Servo esc1;
Servo esc2;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("\n=== Manual Motor Control ==="));
  Serial.println(F("Type: M <motor#> <pwm_us>"));
  Serial.println(F("Example: M 1 1500  →  motor1 = 1500µs"));
  Serial.println(F("         M 2 1600  →  motor2 = 1600µs"));
  Serial.println(F("Valid range: 1000–2000 µs\n"));

  esc1.attach(2);
  esc2.attach(3);

  // Disarm both ESCs
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  Serial.println(F("Motors disarmed at 1000 µs. Waiting for input..."));
}

void loop() {
  // Wait for a command like: M 1 1500
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) return;

    // Expect: M motor value
    if (cmd.startsWith("M") || cmd.startsWith("m")) {
      int motor, pwm;
      int n = sscanf(cmd.c_str(), "M %d %d", &motor, &pwm);

      if (n == 2 && (motor == 1 || motor == 2) && pwm >= 1000 && pwm <= 2000) {
        if (motor == 1) esc1.writeMicroseconds(pwm);
        else             esc2.writeMicroseconds(pwm);

        Serial.print(F("✅ Motor ")); Serial.print(motor);
        Serial.print(F(" set to "));  Serial.print(pwm);
        Serial.println(F(" µs"));
      } else {
        Serial.println(F("⚠️  Usage: M <1|2> <1000–2000>"));
      }
    } else if (cmd.equalsIgnoreCase("STOP")) {
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      Serial.println(F("🛑 Both motors stopped (1000 µs)"));
    } else {
      Serial.println(F("Unknown command. Use M or STOP"));
    }
  }
}
