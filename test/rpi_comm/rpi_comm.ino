#include <RPilink.h>

RPilink link;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  link.begin(Serial); // echo from the same port for manual tests
  Serial.println(F("RPilink Echo: send lines like"));
  Serial.println(F("S,v=0.50,w=-0.25,roll=3.2,lpwm=1510,rpwm=1490,ts=12345*5B"));
}

void loop() {
  if (link.poll()) {
    const RPilink::Packet& p = link.last();

    // Always show raw outcome for monitoring
    Serial.print(F("PARSED valid=")); Serial.print(p.valid);
    Serial.print(F(" cs_ok="));       Serial.print(p.cs_ok);
    Serial.print(F(" v="));           Serial.print(p.v,3);
    Serial.print(F(" w="));           Serial.print(p.w,3);
    Serial.print(F(" roll="));        Serial.print(p.roll,2);
    Serial.print(F(" lpwm="));        Serial.print(p.lpwm);
    Serial.print(F(" rpwm="));        Serial.print(p.rpwm);
    Serial.print(F(" ts="));          Serial.print(p.ts_ms);
    Serial.print(F(" fresh="));       Serial.println(p.fresh(300));
  }
}
