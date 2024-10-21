/*
RRST NHK2025
足回り直進補正
ROS2ノードから受信した入力0~100%に応じてMDに出力する
並列してオドメトリエンコーダーのパルスからX軸、Y軸方向の並進速度[m/s]を計算しROS2ノードに送信する
通信はUDPで行う
PIDはROS2上で処理する
2024/10/21
*/

// 必要なヘッダファイルのインクルード
#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

// 円周率の定義
#define PI 3.141592653589793

// エンコーダー(QEIライブラリ)の設定
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X4_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 2048, QEI::X4_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 2048, QEI::X4_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 2048, QEI::X4_ENCODING);
QEI ENC5(PE_8, PF_9, NC, 2048, QEI::X4_ENCODING);
QEI ENC6(PF_10, PD_11, NC, 2048, QEI::X4_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる
データシート: https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/

using ThisThread::sleep_for;
void receive(UDPSocket *receiver);

// ピン割り当て
// モータードライバのPWM出力ピン
PwmOut MD1P(PA_0);
PwmOut MD2P(PA_3);
PwmOut MD3P(PB_4);
PwmOut MD4P(PB_5);
PwmOut MD5P(PC_7);
PwmOut MD6P(PC_6);
PwmOut MD7P(PC_8);
PwmOut MD8P(PC_9);

// モータードライバの方向制御ピン
DigitalOut MD1D(PD_2);
DigitalOut MD2D(PG_2);
DigitalOut MD3D(PG_3);
DigitalOut MD4D(PE_4);
DigitalOut MD5D(PD_5);
DigitalOut MD6D(PD_6);
DigitalOut MD7D(PD_7);
DigitalOut MD8D(PC_10);

// サーボモーター制御用PWMピン
PwmOut SERVO1(PB_1);
PwmOut SERVO2(PB_6);
PwmOut SERVO3(PD_13);
PwmOut SERVO4(PD_12);

// スイッチ入力ピン
DigitalIn SW1(PF_15);
DigitalIn SW2(PG_14);
DigitalIn SW3(PG_9);
DigitalIn SW4(PE_7);

// グローバル変数の定義
int Pulse[7];      // エンコーダーのパルス格納用
int last_Pulse[7]; // 前回のエンコーダーのパルス格納用（使ってないので削除予定）
float v[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 速度の格納[m/s]
float RPM[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // RPMの格納（使ってないので削除予定）

float period = 10; // 制御周期[ms]
float R = 0.05;    // オムニ直径[mm]
int PPR = 8192;    // エンコーダーのResolution

double mdd[9]; // MDに出力する方向指令を格納
double mdp[9]; // MDに出力するduty比を格納

int main() {
  // 送信データ
  char sendData[32];

  // PWM周波数の設定
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  MD6P.period_us(50);
  MD7P.period_us(50);
  MD8P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz) * 10^3
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */

  // ネットワーク設定
  // 送信先のIPアドレスとポート
  const char *destinationIP = "192.168.8.196";
  const uint16_t destinationPort = 4000;

  // 自機のIPアドレスとポート
  const char *myIP = "192.168.8.215";
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  // イーサネット経由でインターネットに接続するクラス
  EthernetInterface net;
  // IPアドレスとPortの組み合わせを格納しておくクラス
  SocketAddress destination, source, myData;
  // UDP通信関係のクラス
  UDPSocket udp;
  // 受信用スレッド
  Thread receiveThread;

  /* マイコンのネットワーク設定 */
  // DHCPはオフにする（静的にIPなどを設定するため）
  net.set_dhcp(false);
  // IPなど設定
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error >_<\n");
    return -1;
  } else {
    printf("Network connection success ^_^\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);

  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  // メインループ（送信用）
  while (1) {
    using namespace std::chrono;

    // エンコーダーの値を取得
    Pulse[1] = ENC1.getPulses();
    Pulse[2] = ENC2.getPulses();
    Pulse[3] = ENC3.getPulses();
    Pulse[4] = ENC4.getPulses();
    Pulse[5] = ENC5.getPulses();
    Pulse[6] = ENC6.getPulses();

    // エンコーダーのパルスから速度[m/s]を計算
    for (int i = 1; i <= 6; i++) {
      v[i] = Pulse[i] * (R * PI / PPR) * (1000 / period);
    }

    // エンコーダーをリセット
    ENC1.reset();
    ENC2.reset();
    ENC3.reset();
    ENC4.reset();
    ENC5.reset();
    ENC6.reset();

    // 速度データを文字列に変換
    for (int i = 0; i < 7; i++) {
      char temp[32]; // 一時的なバッファ
      sprintf(temp, "%f", v[i]); // floatを文字列に変換

      // sendDataにtempをペーストする
      if (i == 0) {
        strcpy(sendData, temp); // 最初はそのままペースト
      } else {
        strcat(sendData, ","); // 以降カンマ区切りに文字列を連結する
        strcat(sendData, temp);
      }
    }

    // ROS2ノードに現在の速度を送信
    if (const int result = udp.sendto(destination, sendData, sizeof(sendData)) < 0) {
      printf("send Error: %d\n", result); // エラーの処理
    }
    ThisThread::sleep_for(period); // 一定時間待機（制御周期）
  }

  // スレッドの終了を待つ
  receiveThread.join();

  // UDPソケットを閉じ、ネットワーク接続を切断
  udp.close();
  net.disconnect();
  return 0;
}

// UDP受信スレッド関数
void receive(UDPSocket *receiver) {
  using namespace std::chrono;

  SocketAddress source;
  char buffer[64]; // 受信バッファ

  int data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  while (1) {
    memset(buffer, 0, sizeof(buffer));
    if (const int result = receiver->recvfrom(&source, buffer, sizeof(buffer)) < 0) {
      printf("Receive Error : %d", result); // エラーの処理
    } else {
      char *ptr;
      int ptr_counter = 1;
      // カンマを区切りに文字列を分割
      // 1回目
      ptr = strtok(buffer, ",");
      data[1] = atoi(ptr); // charをintに変換

      // 2回目以降
      while (ptr != NULL) {
        ptr_counter++;
        // strtok関数により変更されたNULLのポインタが先頭
        ptr = strtok(NULL, ",");
        if (ptr != NULL) {
          data[ptr_counter] = atoi(ptr); // intに変換
        }
      }

      // 方向と大きさを分離
      for (int i = 1; i <= 8; i++) {
        mdd[i] = (data[i] >= 0) ? 1 : 0;
        mdp[i] = fabs(data[i]) / 100; // 絶対値をとる
      }
    }

    // モータードライバに出力
    MD1D = mdd[1];
    MD2D = mdd[2];
    // MD3D = mdd[3];
    // MD4D = mdd[4];
    MD5D = mdd[3];
    MD6D = mdd[4];
    MD7D = mdd[7];
    MD8D = mdd[8];

    MD1P = mdp[1];
    MD2P = mdp[2];
    // MD3P = mdp[3];
    // MD4P = mdp[4];
    MD5P = mdp[3];
    MD6P = mdp[4];
    MD7P = mdp[7];
    MD8P = mdp[8];
  }
}