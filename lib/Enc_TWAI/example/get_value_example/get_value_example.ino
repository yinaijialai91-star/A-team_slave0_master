#include <Enc_TWAI.h>

#define TX_PIN D0
#define RX_PIN D1

Enc_TWAI motor1;
Enc_TWAI motor2;

void setup() {

  motor1.setup(TX_PIN, RX_PIN, 1);
  motor2.setup(TX_PIN, RX_PIN, 2);
}

void loop() {

  motor1.set_speed(100);
  motor2.set_speed(100);

  delay(3000);

  Serial.printf("motor1 : %d\n", motor1.get_encoder_value());
  Serial.printf("motor2 : %d\n", motor2.get_encoder_value());

  delay(100);

}
