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

  motor1.set_locate(100);
  motor2.set_locate(100);

  delay(3000);

  motor1.set_locate(-100);
  motor2.set_locate(-100);

  delay(3000);

}
