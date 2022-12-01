
#ifndef Cubic_h 
#define Cubic_h
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>

#define SLAVE_ADDRESS_ENCODER1 0xaa //エンコーダのI2Cアドレス
#define SLAVE_ADDRESS_ENCODER2 0xab //arduinoを2つ使う時用

#define ENABLE_PIN 45 //cubicの仕様で、このピンをHIGHにすることによって動作開始

#define MISO 50 //SPI通信に用いるピン
#define MOSI 51
#define SCK 52
#define SS 53 //mega側が53番
#define SS2 49  //反対側が49番ピン、使う可能性は0ではない

extern uint8_t g_buf[10]; //RP2040への送信データを格納する為のint8_t型配列
extern uint8_t g_currentpin[16];

class Cubic_encoder{
    public:
        bool begin(int8_t);
        bool operator >> (int32_t &);
        /*Cubic_encoder(){
          pinMode(ENABLE_PIN,OUTPUT);//Cubicの動作開始
          digitalWrite(ENABLE_PIN,HIGH);
        }*/
        
    private:
        int8_t encoderNum;
        int8_t encoderAddress;
};

class Cubic_motor {
    public:
        void begin(uint8_t); //モータ番号の指定
        bool operator << (int); //データ送信演算子（速度指令）
        

        bool put(int); //値を格納する関数
        static bool send(void); //値をSPI通信で送信する関数


        void check(void); //bufの値をSerial.print()で出力する関数
        int current(void); //電流センサの値を返す関数


        /*Cubic_motor(){
          pinMode(ENABLE_PIN,OUTPUT);//Cubicの動作開始
          digitalWrite(ENABLE_PIN,HIGH);
        }*/

        
    private:
        //書き換えを行う配列要素の指定をするための変数
        uint8_t dutybuf; //duty比を格納する変数
        uint8_t dirbuf; //回転方向を格納する変数

        //モーター番号をここで指定しておく
        uint8_t motornum;

        //RP2040への送信データを格納する配列 staticで定義したが上手くいかなかったので、とりあえずクラス外でグローバル変数として定義した
        //static uint8_t buf[16];


        //SPIのインスタンス
        //SPISettings mySPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
};
#endif
