
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

extern uint8_t buf[10]; //RP2040への送信データを格納する為のint8_t型配列

class Cubic_encoder{
    public:
        bool begin(int8_t);
        bool operator >> (int &);
        Cubic_encoder(){
          pinMode(ENABLE_PIN,OUTPUT);//Cubicの動作開始
          digitalWrite(ENABLE_PIN,HIGH);
        }
        
    private:
        int8_t encoderNum;
        int8_t encoderAddress;
};

class Cubic_motor {
    public:
        void begin(int8_t); //モータ番号の指定
        bool operator << (int); //データ送信演算子（速度指令）
        //bool operator >> (int &); //データ受信演算子(エンコーダ)

        bool put(int); //値を格納する関数
        static bool send(void); //値をSPI通信で送信する関数

        bool check(void);

        Cubic_motor(){
          pinMode(ENABLE_PIN,OUTPUT);//Cubicの動作開始
          digitalWrite(ENABLE_PIN,HIGH);
        }

        
    private:
        //書き換えを行う配列要素の指定をするための変数
        int8_t dutybuf; //duty比を格納するbuf
        int8_t dirbuf; //回転方向を格納するbuf

        //モーター番号をここで指定しておく
        int8_t motornum;

        //RP2040への送信データを格納する配列 staticで定義したが上手くいかなかったので、とりあえずクラス外でグローバル変数として定義した
        //static uint8_t buf[16];


        //SPIのインスタンス
        //SPISettings mySPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
};
#endif
