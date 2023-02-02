#include "Cubic.ver1.8.h"

//#define SLAVE_ADDRESS_ENCODER 0xaa
//#define ENABLE_PIN 2
//SPIピンの番号
//MISO 50
//MOSI 51
//SCK 52
//SS 53

//g_buf[0] = モーター0の方向(1or2)及びパリティー値
//g_buf[1] = モーター1~7の方向が一ビットづつ格納(モーター7の値は最下位2ビット使用 　正回転で0、負回転で
uint8_t g_buf[10];//RP2040への送信データを格納する配列。
//uint8_t g_currentpin[16] = {A0,A3,A6,A5,A10,A8,A14,A12,A13,A15,A9,A11,A1,A7,A4,A2}; //各モータに対応する電流センサの入力アナログピン


SPISettings mySPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);


//初期セットアップを行う関数
void Cubic_motor::begin(uint8_t motornumber){ //制御したいモータを0~7の整数で指定してもらう

    SPI.begin(); //SPI通信セットアップ
    pinMode(SS,OUTPUT); 
    digitalWrite(SS,HIGH);

    pinMode(ENABLE_PIN,OUTPUT);//Cubicの動作開始
    digitalWrite(ENABLE_PIN,HIGH);

    for(int i = 0; i < 10; i++) g_buf[i] = 0;//bufの値を0で初期化
    g_buf[0] = 1; //パリティの値を1で初期化
    
    if(motornumber <8){ //正常な引数値が与えられた時
      this->motornum = motornumber; //モーター番号を保持
      this->dutybuf = motornumber + 2; //まずduty比が格納される要素を指定する。
      
      if(motornumber == 0){ //次にdirが格納される要素を指定する
        this->dirbuf = 0; //パリティ内に値を格納
      }
      else{
        this->dirbuf = 1; //それ以外はすべてduty比の方に
      }
    }
}

//モータを動かす関数 
bool Cubic_motor::operator << (int data){
  this->put(data);
  this->send();
}

//モーターの指示値を代入する関数
bool Cubic_motor::put(int data){
  //buty値を代入
  if(abs(data) > 255) return false; //想定外の入力が来たらエラー
  else if(abs(data) < 3) g_buf[motornum+2] = 0; //パリティとして利用するので2以下の入力は0として出力
  else g_buf[motornum+2] = abs(data); //その他の場合はduty値の絶対値を代入


  //回転方向の値を代入
  //motor0の回転方向はパリティバイトbuf[0]に代入。正回転で1、負回転で2
  if(motornum == 0){ 
    if(data < 0) g_buf[0] = 2;
    else g_buf[0] = 1;
  }

  //motor7の回転方向はbuf[1]の下位2ビットに代入。正回転で0,負回転で1
  else if(motornum == 7){ 
    if(data < 0) g_buf[1] = (g_buf[1] & 0b11111100) | (0b11); 
    else g_buf[1] &= 0b11111100; 
  }

  //その他の場合は最下位から(8-motornum)番目のビットに代入。正回転で0、負回転で1
  else{ 
    if(data < 0) g_buf[1] |= (1<<(8-motornum)); //
    else g_buf[1] &= ((1<<(8-motornum)) ^ 0b11111111); 
  }
}

//配列データを送信する関数
bool Cubic_motor::send(void){
    SPI.beginTransaction(mySPISettings);

    for (int i = 0; i < 10; i++) {//g_buf[]のデータをすべて送る
        digitalWrite(SS,LOW);
        SPI.transfer(g_buf[i]);
        digitalWrite(SS,HIGH);
    }

    SPI.endTransaction(); //通信終了

    delayMicroseconds(1000); //送信毎に1000μsのdelay
}

//シリアルモニターにbufの値を出力する関数
void Cubic_motor::check(void){
  for(int i=0;i<10;i++){
    Serial.print(g_buf[i]);
    Serial.print("  ");
  }
  Serial.println();
}

//電流センサの値を読む関数
int Cubic_motor::current(void){
  return analogRead(g_currentpin[this->motornum]);
}



bool Cubic_encoder::begin(int8_t encodernumber){
  Wire.begin();
  
  pinMode(ENABLE_PIN,OUTPUT);//Cubicの動作開始
  digitalWrite(ENABLE_PIN,HIGH);

  
  //↓二台のarduino nano everyのどちらと通信するかを指定
  if(encodernumber < 0 || encodernumber > 11){
    return false;
  }
  else if(encodernumber < 6){ //引数が0から5の場合
      this->encoderAddress = SLAVE_ADDRESS_ENCODER1; //通信するarduinoのI2Cアドレスを指定
      this->encoderNum = encodernumber; //encoderの番号を指定
  }
  else{ //引数が6から11の場合
      this->encoderAddress = SLAVE_ADDRESS_ENCODER2;
      this->encoderNum = encodernumber-6;
  }
}

bool Cubic_encoder::operator >> (int32_t &enc){
    int32_t rp = 0; //値を一時的に格納する変数
    Wire.beginTransmission(this->encoderAddress); //スレーブへのデータ送信開始
    Wire.write(this->encoderNum); //エンコーダ番号の送信
    Wire.endTransmission(); //送信終了

    Wire.requestFrom(this->encoderAddress, 4); //スレーブから4バイトのデータを受信
    if (Wire.available() >= 4){ //回転数を受信できた時
        for(int i = 0; i < 4; i++) rp += (int32_t)Wire.read() << i*8; //受信したデータをrpに格納
        enc = rp; //encにrpの値を代入
        
        delay(1); //送信毎に1msのdelay（ここに記述するのが正しいかは不明）
        return true;
    }
    else return false; //回転数を受信できなかった場合はfalse
}
