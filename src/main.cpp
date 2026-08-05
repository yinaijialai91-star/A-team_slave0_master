// Aチーム用マスター

#include <Enc_TWAI.h>
#include "driver/twai.h"
#include <Bluepad32.h>

#define TX_PIN 22
#define RX_PIN 21

#define SLAVE1_WHEEL_CONTROL_ID 0x310    // タイヤ
#define SLAVE2_DISHES_ARM_ID 0x710       // お皿
#define SLAVE3_ZEUS_ARM_STS3215_ID 0x410 // 万能手腕
#define SLAVE4_SQUID_ARM_ID 0x110        // いかさん
#define SLAVE5_MARKER_ARM_ID 0x210       // マーカー
#define SLAVEX_BUTSUDAN_LED_ID 0x115     // 仏壇

Enc_TWAI MOTOR1;
Enc_TWAI MOTOR2;
Enc_TWAI MOTOR3;
Enc_TWAI MOTOR4;

unsigned long now = 0, jikan = 0;
uint8_t N = 1 /*移動速度の倍率*/, R = 1 /*万能アームの動作順*/, T_1 = 0, /*皿用アームの動作順*/ T_2 = 8 /*皿用アームコンプレッサー動作順*/, Z = 0;
uint8_t IK = 1 /*いかさんの動作順*/, BALL = 4 /*万能アーム用サーボ昇降機構の動作順*/, MK = 1 /*マーカー用モーター動作順*/, DS = 10;
int MKM = 0 /*マーカー用36GP動作順*/, BBB = 9 /*万能アーム用サーボムツゴロウ動作順*/;

bool task_created = false, IK_moved = false;

int stick_speed_x = 0, stick_speed_y = 0;
int8_t real_speed_x = 0, real_speed_y = 0;
int8_t senkai = 0, real_senkai = 0;
int8_t v1 = 0, v2 = 0, v3 = 0, v4 = 0;

uint8_t mode = 1, wheel_mode = 0;

ControllerPtr myControllers[1];

void onConnectedController(ControllerPtr ctl)
{ // コントローラー接続時のコールバック関数
  bool foundEmptySlot = false;
  if (myControllers[0] == nullptr)
  {
    Serial.printf("CALLBACK: Controller is connected, index=%d\n", 0);
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
    myControllers[0] = ctl;
    foundEmptySlot = true;
  }
  if (!foundEmptySlot)
  {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl)
{ // コントローラー接続解除時のコールバック関数
  bool foundController = false;
  if (myControllers[0] == ctl)
  {
    Serial.printf("CALLBACK: Controller disconnected from index=%d\n", 0);
    myControllers[0] = nullptr;
    foundController = true;
  }
  if (!foundController)
  {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void send(uint16_t ID /*ID*/, int8_t data1 /*識別子*/, int8_t data2 /*データ*/, int8_t data3, int8_t data4, int8_t data5, int8_t data6, int8_t data7, int8_t data8)
{

  twai_message_t SendFrame;

  SendFrame.identifier = ID;
  SendFrame.rtr = 0;
  SendFrame.extd = 0;
  SendFrame.data_length_code = 8;
  SendFrame.data[0] = data1;
  SendFrame.data[1] = data2;
  SendFrame.data[2] = data3;
  SendFrame.data[3] = data4;
  SendFrame.data[4] = data5;
  SendFrame.data[5] = data6;
  SendFrame.data[6] = data7;
  SendFrame.data[7] = data8;

  if (twai_transmit(&SendFrame, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    if (ID != SLAVE1_WHEEL_CONTROL_ID)
    {
      Serial.printf("ID = %x, data1 = %x, data2 = %d, data3 = %d, data4 = %d, data5 = %d, data6 = %d, data7 = %d, data8 = %d\n", ID, data1, data2, data3, data4, data5, data6, data7, data8);
      Serial.println("送信成功");
    }
  }
  else
  {
    if (ID != SLAVE1_WHEEL_CONTROL_ID)
    {
      Serial.printf("ID = %x, data1 = %x, data2 = %d, data3 = %d, data4 = %d, data5 = %d, data6 = %d, data7 = %d, data8 = %d\n", ID, data1, data2, data3, data4, data5, data6, data7, data8);
      Serial.println("送信失敗");
    }
  }
  return;
}

void ctrl(void *pvParameters)
{ // 入力による分岐処理の本体
  for (;;)
  {
    Controller *ctl = static_cast<Controller *>(pvParameters);

    /*******************************共通*******************************/

    stick_speed_x = map(ctl->axisX(), -511, 512, -100, 100); // 左スティックX取得
    stick_speed_y = map(ctl->axisY(), -511, 512, -100, 100); // 左スティックY取得

    if (abs(stick_speed_x) < 5)
    { // X座標デッドゾーン
      stick_speed_x = 0;
    }
    if (abs(stick_speed_y) < 5)
    { // Y座標デッドゾーン
      stick_speed_y = 0;
    }

    int right_round = map(ctl->brake(), 0, 1023, 0, 100);   // ZRボタン
    int left_round = map(ctl->throttle(), 0, 1023, 0, 100); // ZLボタン

    senkai = constrain(right_round - left_round, -100, 100); // 旋回のみ合成

    if (ctl->miscButtons() == 0x04)
    { // 移動速度の倍率変更(プラスボタン)
      if (N < 4)
        N++;
      else
        N = 1;
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (ctl->miscButtons() == 0x02)
    { // 移動速度の倍率変更(マイナスボタン)
      if (N > 1)
        N--;
      else
        N = 4;
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (ctl->buttons() == 0x200)
    { // モード変更(右スティック押し込み)
      if (mode < 4)
      {
        mode++;
      }
      else
      {
        mode = 1;
      }
      printf("now_mode_is %d\n", mode);
      send(SLAVEX_BUTSUDAN_LED_ID, mode, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (ctl->buttons() == 0x100)
    {
      if (wheel_mode < 1)
        wheel_mode++;
      else
        wheel_mode = 0;

      vTaskDelay(pdMS_TO_TICKS(500));
    }

    /******************************************************************/

    /*******************************皿用*******************************/
    if (mode == 1)
    {

      if (ctl->x())
      { // コンプレッサー動作変更
        if (T_2 < 10)
          T_2++;
        else
          T_2 = 9; // 10が停止,9が動作
        send(SLAVE2_DISHES_ARM_ID, T_2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x0001)
      { // 十字上(お皿昇降機構上)(ID2)
        send(SLAVE2_DISHES_ARM_ID, 3, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0002)
      { // 十字下(お皿昇降機構下)(ID2)
        send(SLAVE2_DISHES_ARM_ID, 4, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->y())
      { // Xボタン(お皿昇降機構上)(ID3)
        send(SLAVE2_DISHES_ARM_ID, 6, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->a())
      { // Bボタン(お皿昇降機構下)(ID3)
        send(SLAVE2_DISHES_ARM_ID, 7, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0004)
      { // 十字右(お皿くるくる上)
        send(SLAVE2_DISHES_ARM_ID, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0008)
      { // 十字左(お皿くるくる下)
        send(SLAVE2_DISHES_ARM_ID, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else
      { // 停止処理
        send(SLAVE2_DISHES_ARM_ID, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      if (ctl->b())
      {
        if (!IK_moved)
        {
          send(SLAVE5_MARKER_ARM_ID, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
          IK_moved = true;
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (DS < 13)
          DS++;
        else
          DS = 11;
        send(SLAVE2_DISHES_ARM_ID, DS, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }

    /******************************************************************/

    /*****************************万能手腕*****************************/
    if (mode == 2)
    {
      if (ctl->x())
      { // 万能アーム動作（仮
        if (R < 5)
          R++;
        else
          R = 4;
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 3, R, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->y())
      { // 万能アーム用昇降機構、下
        MOTOR1.set_speed_stable(-160);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->a())
      { // 万能アーム用昇降機構、上
        MOTOR1.set_speed_stable(160);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (!ctl->y() && !ctl->a())
      { // 万能アーム用昇降機構、停止
        MOTOR1.set_speed_stable(0);
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      if (ctl->dpad() == 0x0001)
      { // 十字上（万能アーム用サーボ昇降機構)
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 4, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0002)
      { // 十字下(万能アーム用サーボ昇降機構)
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 4, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }

      if (ctl->dpad() == 0x0004)
      { // 十字右(万能アーム用ラック伸ばし)
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 7, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0008)
      { // 十字左(万能アーム用ラック縮め)
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 8, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(5));
      }

      if (ctl->buttons() == 0x20)
      { // ちょい離し(Rボタン)
        if (BBB < 14)
        {
          BBB++;
        }
        else
        {
          BBB = 10;
        }
        send(SLAVE3_ZEUS_ARM_STS3215_ID, BBB, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      else if (ctl->buttons() == 0x10)
      { // 掴む(Lボタン)
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 3, 4, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->b())
      { // 万能アーム用サーボ昇降機構、便利機能
        if (BALL < 6)
          BALL++;
        else
          BALL = 5;
        send(SLAVE3_ZEUS_ARM_STS3215_ID, BALL, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }
    /*****************************いかさん*****************************/

    if (mode == 3)
    {

      if (ctl->x())
      {
        if (IK > 3)
          IK = 2;
        else
          IK++;
        send(SLAVE4_SQUID_ARM_ID, IK, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->y())
      {
        MOTOR2.set_speed(60);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->a())
      {
        MOTOR2.set_speed(-60);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (!ctl->y() && !ctl->a())
      {
        MOTOR2.set_speed(0);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    // 初期位置ー＞棒伸ばすー＞ハンド展開ー＞ハンド掴む
    // 棒伸ばすー＞ハンド離す

    /******************************************************************/

    /***************************マーカー*******************************/
    if (mode == 4)
    {

      if (ctl->dpad() == 0x01)
      { // 釣り竿伸び(十字上)
        MOTOR3.set_speed_stable(255);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x02)
      { // 釣り竿縮み(十字下)
        MOTOR3.set_speed_stable(-255);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else
      {
        MOTOR3.set_speed_stable(0);
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      if (ctl->dpad() == 0x08)
      { // ラック伸び(十字左)
        MOTOR4.set_speed_stable(100);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      else if (ctl->dpad() == 0x04)
      { // ラック縮み(十字右)
        MOTOR4.set_speed_stable(-100);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      else
      {
        MOTOR4.set_speed_stable(0);
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      if (ctl->buttons() == 0x20)
      { // サーボ左旋回(おそらくLボタン)
        send(SLAVE5_MARKER_ARM_ID, 3, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      else if (ctl->buttons() == 0x10)
      { // サーボ右旋回(おそらくRボタン)
        send(SLAVE5_MARKER_ARM_ID, 4, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      /******************************************************************/
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void vector_task(void *pvParameters)
{ // タイヤのベクトル計算専用タスク
  for (;;)
  {

    //       1     2
    //      "\\"_"//"
    //       |     |
    //      "//"_"\\"
    //       3     4

    BP32.update();

    now = millis();

    if (now - jikan >= 50)
    {
      jikan = now;

      if (real_speed_x < stick_speed_x)
      {
        real_speed_x += 5;
      }
      if (real_speed_y < stick_speed_y)
      {
        real_speed_y += 5;
      }

      if (real_speed_x > stick_speed_x)
      {
        real_speed_x -= 5;
      }
      if (real_speed_y > stick_speed_y)
      {
        real_speed_y -= 5;
      }

      if (real_senkai < senkai)
      {
        real_senkai += 5;
      }
      if (real_senkai > senkai)
      {
        real_senkai -= 5;
      }
    }

    // int8_t v1 = constrain(real_speed_y - real_speed_x + senkai, -100, 100);
    // int8_t v2 = constrain(real_speed_y + real_speed_x - senkai, -100, 100);
    // int8_t v3 = constrain(real_speed_y + real_speed_x + senkai, -100, 100);
    // int8_t v4 = constrain(real_speed_y - real_speed_x - senkai, -100, 100);

    if (wheel_mode == 0)
    {
      v1 = constrain(real_speed_y - real_speed_x + senkai, -100, 100);
      v2 = constrain(real_speed_y + real_speed_x - senkai, -100, 100);
      v3 = constrain(real_speed_y + real_speed_x + senkai, -100, 100);
      v4 = constrain(real_speed_y - real_speed_x - senkai, -100, 100);
    }
    else if (wheel_mode == 1)
    {
      v1 = constrain(-real_speed_y + real_speed_x + senkai, -100, 100);
      v2 = constrain(-real_speed_y - real_speed_x - senkai, -100, 100);
      v3 = constrain(-real_speed_y - real_speed_x + senkai, -100, 100);
      v4 = constrain(-real_speed_y + real_speed_x - senkai, -100, 100);
    }

    send(SLAVE1_WHEEL_CONTROL_ID, 1, N, v1, v2, v3, v4, 0xAA, 0xAA);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup()
{

  Serial.begin(115200);

  vTaskDelay(pdMS_TO_TICKS(200));

  // /***********************************CAN関連********************************************/
  // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  // if (ret == ESP_OK) Serial.println("インストール完了");
  // else Serial.println("インストール失敗");
  // ret = twai_start();
  // if (ret == ESP_OK) Serial.println("CANスタート完了");
  // else Serial.println("CANスタート失敗");
  // /**************************************************************************************/

  /************************************36GPMotor*****************************************/

  MOTOR1.setup(22, 21, 1); // 万能手腕
  MOTOR2.setup(22, 21, 2); // いかさん
  MOTOR3.setup(22, 21, 3); // マーカー
  MOTOR4.setup(22, 21, 4); // マーカー

  /**************************************************************************************/

  /************************************Bluepad32*****************************************/

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  /**************************************************************************************/
}

void loop()
{ // コントローラーと接続を確立させた後にタスクを作成、looptaskを削除

  bool dataUpdated = BP32.update();
  if (!task_created && dataUpdated && myControllers[0]->isConnected() && myControllers[0] != nullptr)
  {

    task_created = true;

    xTaskCreateUniversal(ctrl, "ctrltask", 8192, myControllers[0], 2, nullptr, APP_CPU_NUM);
    xTaskCreateUniversal(vector_task, "vector_Task", 8192, nullptr, 1, nullptr, APP_CPU_NUM);

    vTaskDelete(NULL);
  }

  vTaskDelay(pdMS_TO_TICKS(1));
}
