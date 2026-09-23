// Aチーム用マスター

#include <Enc_TWAI.h>    //36GP-3650制御用
#include "driver/twai.h" //CAN通信用
#include <Bluepad32.h>   //無線コン制御用
#include <uni.h>         //MACホワイトリスト作成用

#define TX_PIN 22 // CAN用TXピン
#define RX_PIN 21 // CAN用RXピン

#define SLAVE1_WHEEL_CONTROL_ID 0x310    // タイヤ
#define SLAVE2_DISHES_ARM_ID 0x710       // お皿
#define SLAVE3_ZEUS_ARM_STS3215_ID 0x410 // 万能手腕
#define SLAVE4_SQUID_ARM_ID 0x110        // いかさん
#define SLAVE5_MARKER_ARM_ID 0x210       // マーカー
#define SLAVEX_BUTSUDAN_LED_ID 0x115     // 仏壇

Enc_TWAI MOTOR1; // 万能手腕
Enc_TWAI MOTOR2; // いかさん

static const char *controller_addr_string = "98:B6:EA:96:93:4B";

unsigned long now = 0, jikan = 0;
uint8_t N = 2;          /*移動速度の倍率*/
uint8_t R = 1;          /*万能アームの動作順*/
uint8_t T_1 = 0;        /*皿用アームの動作順*/
uint8_t T_2 = 8;        /*皿用アームコンプレッサー動作順*/
uint8_t IK_taosu = 0;   /*いかさんくるくる動作順*/
uint8_t IK_hand = 0;    /*いかさん用ハンド動作順*/
uint8_t IK_houyou = 0;  /*いかさん抱擁動作準*/
uint8_t BALL_FIRST = 0; /*万能アーム目標ボール抽選*/
uint8_t BALL = 4;       /*万能アーム用サーボ昇降機構の動作順*/
uint8_t MK = 1;         /*マーカー用モーター動作順*/
uint8_t DS = 10;        /*皿用便利機能動作順*/
uint8_t MKM = 0;        /*マーカー動作順*/
uint8_t BBB = 9;        /*万能アーム用サーボムツゴロウ動作順*/
uint8_t past_duty = 0;  /*過去の倍率*/
bool teisoku = false;   /*低速モード入切*/

bool task_created = false, IK_moved = false;
bool motor1_stopped = false, motor2_stopped = false;
bool m2006_stopped = false;

int stick_speed_x = 0, stick_speed_y = 0;
int8_t real_speed_x = 0, real_speed_y = 0;
int8_t senkai = 0, real_senkai = 0;
int8_t v1 = 0, v2 = 0, v3 = 0, v4 = 0; /*各タイヤのベクトル情報格納用関数*/

uint8_t mode = 0, wheel_mode = 0;

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

    stick_speed_x = map(ctl->axisX(), -511, 512, -50, 50); // 左スティックX取得
    stick_speed_y = map(ctl->axisY(), -511, 512, -50, 50); // 左スティックY取得

    if (abs(stick_speed_x) < 5)
    { // X座標デッドゾーン
      stick_speed_x = 0;
    }
    if (abs(stick_speed_y) < 5)
    { // Y座標デッドゾーン
      stick_speed_y = 0;
    }

    int right_round = map(ctl->brake(), 0, 1023, 0, 60);   // ZRボタン
    int left_round = map(ctl->throttle(), 0, 1023, 0, 60); // ZLボタン

    senkai = constrain(right_round - left_round, -60, 60); // 旋回のみ合成

    if (ctl->miscButtons() == 0x04)
    { // 移動速度の倍率変更(プラスボタン)
      if (N < 4)
        N += 2;
      else
        N = 2;
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (ctl->miscButtons() == 0x02)
    { // 移動速度の倍率変更(マイナスボタン)
      if (!teisoku)
      {
        teisoku = true;
        past_duty = N;
        N = 1;
      }
      else
      {
        teisoku = false;
        N = past_duty;
      }
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
        mode = 2;
      }
      printf("now_mode_is %d\n", mode);
      send(SLAVEX_BUTSUDAN_LED_ID, mode, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      vTaskDelay(pdMS_TO_TICKS(300));
      if (mode == 3)
      {
        send(SLAVE4_SQUID_ARM_ID, 5, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
      }
    }

    /******************************************************************/

    /*******************************初期*******************************/

    if (mode == 0)
    {

      wheel_mode = 0; // 前が前になるようにする

      if (ctl->dpad() == 0x01)
      { // ハンド開く(十字上)
        send(SLAVE5_MARKER_ARM_ID, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x02)
      { // ハンド閉じる(十字下)
        send(SLAVE5_MARKER_ARM_ID, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    /******************************************************************/

    /*******************************皿用*******************************/
    if (mode == 1)
    {
      wheel_mode = 0; // 前が前になるようにする

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
        m2006_stopped = false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0002)
      { // 十字下(お皿昇降機構下)(ID2)
        send(SLAVE2_DISHES_ARM_ID, 4, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        m2006_stopped = false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->y())
      { // Xボタン(お皿昇降機構上)(ID3)
        send(SLAVE2_DISHES_ARM_ID, 6, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        m2006_stopped = false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->a())
      { // Bボタン(お皿昇降機構下)(ID3)
        send(SLAVE2_DISHES_ARM_ID, 7, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        m2006_stopped = false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0004)
      { // 十字右(お皿くるくる上)
        send(SLAVE2_DISHES_ARM_ID, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        m2006_stopped = false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() == 0x0008)
      { // 十字左(お皿くるくる下)
        send(SLAVE2_DISHES_ARM_ID, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        m2006_stopped = false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->dpad() != 0x01 && ctl->dpad() != 0x02 && ctl->dpad() != 0x04 && ctl->dpad() != 0x08 && !m2006_stopped)
      { // 停止処理
        m2006_stopped = true;
        send(SLAVE2_DISHES_ARM_ID, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(50));
        send(SLAVE2_DISHES_ARM_ID, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      if (ctl->b()) // 皿用便利機能
      {
        if (DS < 13)
          DS++;
        else
          DS = 11;

        if (DS == 12)
        {
          send(SLAVE4_SQUID_ARM_ID, 1, 3, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        }
        if (DS == 13)
        {
          send(SLAVE4_SQUID_ARM_ID, 1, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        }
        send(SLAVE2_DISHES_ARM_ID, DS, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    /******************************************************************/

    /*****************************いかさん*****************************/
    if (mode == 2)
    {

      wheel_mode = 2; // いかさんが前になるようにする

      if (!teisoku)
      {
        teisoku = true;
        past_duty = N;
        N = 1;
      }

      if (ctl->x())
      { // いかさん倒す
        if (IK_taosu < 2)
          IK_taosu++;
        else
          IK_taosu = 1;
        send(SLAVE4_SQUID_ARM_ID, 1, IK_taosu, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->b())
      { // いかさんハンド開閉
        if (IK_hand < 6)
          IK_hand++;
        else
          IK_hand = 2;

        send(SLAVE4_SQUID_ARM_ID, 3, IK_hand, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x01)
      {
        if (IK_houyou < 2)
          IK_houyou++;
        else
          IK_houyou = 1;

        send(SLAVE4_SQUID_ARM_ID, 2, IK_houyou, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x08)
      {
        send(SLAVE4_SQUID_ARM_ID, 5, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(300));
      }

      if (ctl->dpad() == 0x04)
      {
        send(SLAVE4_SQUID_ARM_ID, 5, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(300));
      }

      /***********いかさん昇降機構***********/
      if (ctl->y())
      { /*いかさん上昇*/
        MOTOR2.set_speed_stable(-255);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (ctl->a())
      { // いかさん下降
        MOTOR2.set_speed_stable(255);
        vTaskDelay(pdMS_TO_TICKS(5));
      }
      else if (!ctl->y() && !ctl->a())
      { // いかさん昇降機構停止
        MOTOR2.set_speed_stable(0);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }

    /******************************************************************/

    /*****************************万能手腕*****************************/
    if (mode == 3)
    {
      wheel_mode = 3; // 万能手腕が前になるようにする

      if (teisoku)
      {
        teisoku = false;
        N = past_duty;
      }

      if (ctl->b())
      { // 万能アーム動作(ボール用)
        if (R < 5)
        {
          R++;
        }
        else
        {
          R = 2;
          BALL_FIRST = (BALL_FIRST == 1) ? 0 : 1;
        }
        send(SLAVE3_ZEUS_ARM_STS3215_ID, 3, R, BALL_FIRST, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->y())
      { // 万能アーム用昇降機構、下
        MOTOR1.set_speed_stable(-240);
        vTaskDelay(pdMS_TO_TICKS(5));
        motor1_stopped = false;
      }
      else if (ctl->a())
      { // 万能アーム用昇降機構、上
        MOTOR1.set_speed_stable(240);
        vTaskDelay(pdMS_TO_TICKS(5));
        motor1_stopped = false;
      }
      else if (!ctl->y() && !ctl->a() && !motor1_stopped)
      { // 万能アーム用昇降機構、停止
        MOTOR1.set_speed_stable(0);
        vTaskDelay(pdMS_TO_TICKS(10));
        MOTOR1.set_speed_stable(0);
        motor1_stopped = true;
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
      { // 万能手腕動作(Rボタン)(シャトル用)
        if (BBB < 13)
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

      if (ctl->x())
      { // 万能アーム用サーボ昇降機構、便利機能
        if (BALL < 6)
          BALL++;
        else
          BALL = 5;
        send(SLAVE3_ZEUS_ARM_STS3215_ID, BALL, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    /******************************************************************/

    /*****************************マーカー*****************************/
    if (mode == 4)
    {

      wheel_mode = 1; // バックするようにする

      if (ctl->b())
      { // サーボ倒す
        send(SLAVE4_SQUID_ARM_ID, 4, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x01)
      { // ハンド開く(十字上)
        send(SLAVE5_MARKER_ARM_ID, 1, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x02)
      { // ハンド閉じる(十字下)
        send(SLAVE5_MARKER_ARM_ID, 2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->buttons() == 0x20)
      { // 電磁弁開放(Rボタン)
        send(SLAVE2_DISHES_ARM_ID, 15, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA);
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (ctl->dpad() == 0x08)
      {
      }

      /******************************************************************/
    }

    /******************************************************************/

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

    if (now - jikan >= 10)
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

    if (wheel_mode == 0)
    { // お皿
      v1 = constrain(real_speed_y - real_speed_x + senkai, -100, 100);
      v2 = constrain(real_speed_y + real_speed_x - senkai, -100, 100);
      v3 = constrain(real_speed_y + real_speed_x + senkai, -100, 100);
      v4 = constrain(real_speed_y - real_speed_x - senkai, -100, 100);
    }
    else if (wheel_mode == 1)
    { // ただのバック
      v1 = constrain(-real_speed_y + real_speed_x + senkai, -100, 100);
      v2 = constrain(-real_speed_y - real_speed_x - senkai, -100, 100);
      v3 = constrain(-real_speed_y - real_speed_x + senkai, -100, 100);
      v4 = constrain(-real_speed_y + real_speed_x - senkai, -100, 100);
    }
    else if (wheel_mode == 2)
    { // いかさん
      v1 = constrain(-real_speed_y - real_speed_x + senkai, -100, 100);
      v2 = constrain(real_speed_y - real_speed_x - senkai, -100, 100);
      v3 = constrain(real_speed_y - real_speed_x + senkai, -100, 100);
      v4 = constrain(-real_speed_y - real_speed_x - senkai, -100, 100);
    }
    else if (wheel_mode == 3)
    { // 万能手腕
      v1 = constrain(real_speed_y + real_speed_x + senkai, -100, 100);
      v2 = constrain(-real_speed_y + real_speed_x - senkai, -100, 100);
      v3 = constrain(-real_speed_y + real_speed_x + senkai, -100, 100);
      v4 = constrain(real_speed_y + real_speed_x - senkai, -100, 100);
    }

    send(SLAVE1_WHEEL_CONTROL_ID, 1, N, v1, v2, v3, v4, 0xAA, 0xAA);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup()
{

  Serial.begin(115200);

  vTaskDelay(pdMS_TO_TICKS(200));

  /************************************36GPMotor*****************************************/

  MOTOR1.setup(22, 21, 1); // 万能手腕
  MOTOR2.setup(22, 21, 2); // いかさん

  /**************************************************************************************/

  /************************************Bluepad32*****************************************/

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  bd_addr_t controller_addr;
  sscanf_bd_addr(controller_addr_string, controller_addr);
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);
  // uni_bt_allowlist_remove_all();

  /**************************************************************************************/
}

void loop()
{ // コントローラーと接続を確立させた後にタスクを作成、looptaskを削除

  bool dataUpdated = BP32.update();
  if (!task_created && dataUpdated && myControllers[0] != nullptr && myControllers[0]->isConnected())
  {

    task_created = true;

    xTaskCreateUniversal(ctrl, "ctrltask", 8192, myControllers[0], 2, nullptr, APP_CPU_NUM);
    xTaskCreateUniversal(vector_task, "vector_Task", 8192, nullptr, 1, nullptr, APP_CPU_NUM);

    vTaskDelete(NULL);
  }

  vTaskDelay(pdMS_TO_TICKS(1));
}
