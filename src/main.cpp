#include <M5Core2.h>
// #include <M5Stack.h>
// #include <M5StickC.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 26

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// LEDを点滅させる(RCCHECKマクロでエラー判定されたときに実行)
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

uint16_t getColor(uint8_t red, uint8_t green, uint8_t blue){
  return ((red>>3)<<11) | ((green>>2)<<5) | (blue>>3);
}

// メッセージを受信したときに実行するコールバック関数
void subscription_callback(const void * msgin)
{
  // 受信したメッセージを格納
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  // 受信値を0.0~0.99の範囲に収める
  float duty = max(min(msg->data, 0.99f), 0.0f);

  // LEDを光らせる
  ledcWrite(0, (int)(duty * 255));

  // 画面を指定した明るさの白色で更新
  M5.Lcd.fillScreen(getColor((uint)(255*duty), (uint)(255*duty), (uint)(255*duty)));
}

void setup() {

  // M5Stackのセットアップ
  M5.begin();

  // 画面の明るさを最大にして白色に塗りつぶす
  M5.Lcd.setBrightness(255);
  M5.Lcd.fillScreen(WHITE);

  // PWM波生成に使用するタイマーのセットアップ
  // 引数は チャンネル, 周波数, 分解能(bit)
  ledcSetup(0, 1000, 8);

  // ピンにタイマーを割り当てる
  // 引数は ピン番号, チャンネル
  ledcAttachPin(LED_PIN, 0);

  // micro-ROSをセットアップする関数
  // Wi-FI経由で通信する際の引数はSSID, パスワード, 接続先のPCのIPアドレス, ポート番号(適当でOK)
  set_microros_wifi_transports("SSID", "PASSWORD", "IP ADDRESS", 8888);

  // USB経由で通信する場合
  //set_microros_transports();

  // セットアップが完了するまで少しの間待機
  delay(2000);

  // allocatorを宣言
  allocator = rcl_get_default_allocator();

  // 通信機能をサポートする構造体を宣言
  // 引数は サポートの構造体, argc, argv, allocator
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // nodeを作成
  // 引数は nodeの構造体, nodeの名前, 名前空間, サポートの構造体
  RCCHECK(rclc_node_init_default(&node, "led_node", "", &support));

  // subscriptionを作成
  // 引数は subscriptionの構造体, nodeの構造体, メッセージサポート, topicの名前
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "duty"));

  // executorを作成
  // 引数は executorの構造体, サポートのコンテクスト, ハンドル数, allocator
  // ハンドル数：マイコンで処理するtopic, service, timerなどのコールバックの数
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // executorにsubscriptionを追加
  // 引数は executor, subscription, msg, コールバック関数, 
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  
  // 5ミリ秒間待機
  delay(100);

  // 実行可能なハンドル(コールバック関数)があれば実行する
  // 指定した待機時間が経過した後に制御が返される
  // 引数は executor, 待機時間(ナノ秒)
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}