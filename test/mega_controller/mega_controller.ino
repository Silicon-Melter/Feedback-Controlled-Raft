#include <FlyskyReader.h>

// Use Serial1 for iBus
FlyskyReader receiver(Serial1);

void setup() {
  Serial.begin(115200);
  receiver.begin();
}

void loop() {
  for (byte i = 0; i < 4; i++) {
    int value = receiver.readChannel(i, -100, 100, 0);
    Serial.print("Ch");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" | ");
  }

  Serial.print("Ch5: ");
  Serial.print(receiver.readSwitch(4, false));
  Serial.print(" Ch6: ");
  Serial.println(receiver.readSwitch(5, false));

  delay(10);
}
