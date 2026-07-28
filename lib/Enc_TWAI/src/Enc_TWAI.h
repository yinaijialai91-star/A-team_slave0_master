#ifndef Enc_TWAI_H_
#define Enc_TWAI_H_

#include <Arduino.h>
#include "driver/twai.h"

class Enc_TWAI
{
public:

    void setup(int TX_PIN, int RX_PIN, int ID);

    void set_speed(int16_t speed);

    void set_speed_stable(int16_t speed);

    void set_locate(int16_t locate);

    int get_encoder_value();


private:
    uint8_t my_ID = 0;
    
    static bool init_flag;
    static Enc_TWAI *instance_map[256];
    static TaskHandle_t receive_task_handle;

    int encoder_value = 0;

    static void receive_task(void *pvParameters);

    int parse_feedback(twai_message_t *receiveframe);
};

#endif