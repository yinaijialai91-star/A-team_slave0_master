#include <Arduino.h>
#include "driver/twai.h"
#include "Enc_TWAI.h"

bool Enc_TWAI::init_flag = false;
Enc_TWAI *Enc_TWAI::instance_map[256] = {nullptr};
TaskHandle_t Enc_TWAI::receive_task_handle = nullptr;

void Enc_TWAI::setup(int TX_PIN, int RX_PIN, int ID)
{

    instance_map[ID] = this;
    my_ID = ID;

    if (!init_flag)
    {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
        {
            Serial.printf("TWAIドライバのインストールに成功しました\n");
        }
        else
        {
            Serial.printf("TWAIドライバのインストールに失敗しました\n");
            return;
        }

        if (twai_start() == ESP_OK)
        {
            Serial.printf("TWAIドライバの開始に成功しました\n");
        }
        else
        {
            Serial.printf("TWAIドライバの開始に失敗しました\n");
            return;
        }

        xTaskCreate(receive_task, "receive_task", 4096, NULL, 2, &receive_task_handle);

        init_flag = true;
    }
}

void Enc_TWAI::set_speed(int16_t speed)
{
    twai_message_t sendframe;

    sendframe.identifier = 0x50;    // 宛先のID
    sendframe.extd = 0;             // IDを11bitで構成するか29bitで構成するか(基本０でいい)
    sendframe.rtr = 0;              // リモートフレームにするか(基本０でいい)
    sendframe.data_length_code = 8; // 送信するデータの長さ
    sendframe.data[0] = my_ID;
    sendframe.data[1] = 1;
    sendframe.data[2] = (speed >> 8) & 0xFF;
    sendframe.data[3] = (speed & 0xFF);
    sendframe.data[4] = 0xAA;
    sendframe.data[5] = 0xAA;
    sendframe.data[6] = 0xAA;
    sendframe.data[7] = 0xAA;

    esp_err_t ret = twai_transmit(&sendframe, pdMS_TO_TICKS(10));
    if (ret == ESP_OK)
    {
        //Serial.printf("送信成功\n");
    }
    else
    {
        //Serial.printf("送信失敗\n");
    }
}

void Enc_TWAI::set_speed_stable(int16_t speed)
{
    twai_message_t sendframe;

    sendframe.identifier = 0x50;    // 宛先のID
    sendframe.extd = 0;             // IDを11bitで構成するか29bitで構成するか(基本０でいい)
    sendframe.rtr = 0;              // リモートフレームにするか(基本０でいい)
    sendframe.data_length_code = 8; // 送信するデータの長さ
    sendframe.data[0] = my_ID;
    sendframe.data[1] = 4;
    sendframe.data[2] = (speed >> 8) & 0xFF;
    sendframe.data[3] = (speed & 0xFF);
    sendframe.data[4] = 0xAA;
    sendframe.data[5] = 0xAA;
    sendframe.data[6] = 0xAA;
    sendframe.data[7] = 0xAA;

    esp_err_t ret = twai_transmit(&sendframe, pdMS_TO_TICKS(10));
    if (ret == ESP_OK)
    {
        //Serial.printf("送信成功\n");
    }
    else
    {
        //Serial.printf("送信失敗\n");
    }
}

void Enc_TWAI::set_locate(int16_t locate)
{
    twai_message_t sendframe;

    sendframe.identifier = 0x50;    // 宛先のID
    sendframe.extd = 0;             // IDを11bitで構成するか29bitで構成するか(基本０でいい)
    sendframe.rtr = 0;              // リモートフレームにするか(基本０でいい)
    sendframe.data_length_code = 8; // 送信するデータの長さ
    sendframe.data[0] = my_ID;
    sendframe.data[1] = 2;
    sendframe.data[2] = (locate >> 8) & 0xFF;
    sendframe.data[3] = (locate & 0xFF);
    sendframe.data[4] = 0xAA;
    sendframe.data[5] = 0xAA;
    sendframe.data[6] = 0xAA;
    sendframe.data[7] = 0xAA;

    esp_err_t ret = twai_transmit(&sendframe, pdMS_TO_TICKS(10));
    if (ret == ESP_OK)
    {
        //Serial.printf("送信成功\n");
    }
    else
    {
        //Serial.printf("送信失敗\n");
    }
}

int Enc_TWAI::get_encoder_value()
{
    twai_message_t sendframe;

    sendframe.identifier = 0x50;    // 宛先のID
    sendframe.extd = 0;             // IDを11bitで構成するか29bitで構成するか(基本０でいい)
    sendframe.rtr = 0;              // リモートフレームにするか(基本０でいい)
    sendframe.data_length_code = 8; // 送信するデータの長さ
    sendframe.data[0] = my_ID;
    sendframe.data[1] = 3;
    sendframe.data[2] = 0xAA;
    sendframe.data[3] = 0xAA;
    sendframe.data[4] = 0xAA;
    sendframe.data[5] = 0xAA;
    sendframe.data[6] = 0xAA;
    sendframe.data[7] = 0xAA;

    twai_transmit(&sendframe, pdMS_TO_TICKS(10));

    return this->encoder_value; // parse_feedback関数で更新された値を返す

}

int Enc_TWAI::parse_feedback(twai_message_t *receiveframe)
{
    int16_t feedback_value = (receiveframe->data[2] << 8) | receiveframe->data[3];

    this->encoder_value = feedback_value;

    return feedback_value;

}

void Enc_TWAI::receive_task(void *pvParameters)
{
    twai_message_t receiveframe;
    for (;;)
    {
        if (twai_receive(&receiveframe, portMAX_DELAY) == ESP_OK)
        {
            uint8_t id = receiveframe.data[0];
            if (instance_map[id] != nullptr)
            {
                instance_map[id]->parse_feedback(&receiveframe);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
