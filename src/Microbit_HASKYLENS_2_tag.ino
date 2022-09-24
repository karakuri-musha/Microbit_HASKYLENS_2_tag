#include "HUSKYLENS.h"                                            // HUSKYLENS制御ライブラリ
#include "LEDMtrix_Code.h"                                        // LEDマトリックス用BMP定義
#include <Adafruit_Microbit.h>                                    // micro:bitのLEDマトリックス制御ライブラリ

HUSKYLENS huskylens;                                              // HUSKYLENSの生成

Adafruit_Microbit_Matrix mb_matrix;                               // micro:bitのLEDマトリックス生成

const int buttonA = 5;                                            // A ボタン GPIO pin
const int buttonB = 11;                                           // B ボタン GPIO pin

const int motor_right_pwm = 13;                                   // motor_Right GPIO pin
const int motor_left_pwm  = 14;                                   // motor_left GPIO pin
const int motor_right_on = 15;                                    // motor_Right on/off switch
const int motor_left_on = 16;                                     // motor_Left on/off switch
                                                    
void printResult(HUSKYLENSResult result);                         // シリアルへの認識結果出力関数

int left = 0, right = 0;                                          // 左、右モータの速度（PWM）格納用
int setup_stat = 1;                                               // 動作モード（学習／実行）の設定用
int tagid = 1;                                                    // 設定用TagIDの格納用
int findtag = 0;                                                  // 認識したTagIDの格納用

// -------------------------------------------------------------
// PWM 制御によるモーター制御（ 前進　1～1023  後進　-1～-1023）
// -------------------------------------------------------------
void tamiya_robot_speed (int16_t left_speed, int16_t right_speed) {
  digitalWrite (motor_right_on, LOW);                             // 右モーターをON状態にする
  digitalWrite (motor_left_on, LOW);                              // 左モーターをON状態にする
  analogWrite (motor_right_pwm, right_speed);                     // 右モーターのスピードを設定
  analogWrite (motor_left_pwm, left_speed);                       // 左モーターのスピードを設定
  
  if (findtag > 1) {                                              // 前進以外（右、左）の場合、回転時間待機
    delay(1750);
  }
}

// -------------------------------------------------------------
// 初期化処理
// -------------------------------------------------------------
void setup() {  
  Serial.begin(9600);                                             // シリアル接続の開始
  Serial.println("microbit is ready!");                           // シリアルコンソールへの出力
  
  mb_matrix.begin();                                              // LEDマトリックス制御の開始
  mb_matrix.print("HELLO!");                                      // LEDマトリックスに"HELLO！"をスクロール表示

  pinMode(buttonA, INPUT);                                        // micro:bitのAボタン用GPIOを入力に設定
  pinMode(buttonB, INPUT);                                        // micro:bitのBボタン用GPIOを入力に設定
  
  pinMode(motor_right_on, OUTPUT);                                // 右モーターのON/OFF状態選択用GPIOを出力に設定
  pinMode(motor_left_on, OUTPUT);                                 // 左モーターのON/OFF状態選択用GPIOを出力に設定
  digitalWrite(motor_right_on, HIGH);                             // 右モーターを停止状態に設定
  digitalWrite(motor_left_on, HIGH);                              // 左モーターを停止状態に設定

  Wire.begin();                                                   // I2C バスに接続
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));                           // HUSKYLENSとI2C接続を開始できない場合
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);            // アルゴリズムをTag Recognitionに変更
  if (!huskylens.request(1)) {
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    mb_matrix.print("ERR-1"); }
}

// -------------------------------------------------------------
// 繰り返し処理
// -------------------------------------------------------------
void loop(){
  if (! digitalRead(buttonA)) {                                  
    if (setup_stat == 1) {                                        // 起動後モード選択で　Aボタン押下時：再学習モード（リセット）
      setup_stat = 2;                                             // 動作モードを学習モードに変更
      huskylens.writeForget();                                    // HUSKYLENSのモデルをリセット
      mb_matrix.show(A_bmp);                                      // microbit のLEDマトリックスに「A」を表示
      delay(1000);
      mb_matrix.print("SETUP MODE");                              // microbit のLEDマトリックスに設定モードであると表示

      mb_matrix.show(foward_bmp);                                 // 1つ目の学習項目（前進）をLEDマトリックスに表示
    } 
    else if (setup_stat == 2) {
      switch (tagid) {                                            // 学習モード中にAボタンが押された場合
        case 1:                                                   // Tag 1 の学習結果を保存
          while (!huskylens.writeLearn(1)) {
            Serial.println(F("learn object Tag1 failed!")); 
            delay(100);
          }
          mb_matrix.print("TAG1 OK");                             
          delay(2000);                                            
          tagid = ++tagid;                                        // 次のタグに進む
          mb_matrix.show(right_bmp);                              // Tag2 （右矢印）をLEDマトリックスに表示
          break;
        case 2:                                                   // Tag2 の学習結果を保存
          while(!huskylens.writeLearn(2)) {
            Serial.println(F("learn object Tag2 failed!")); 
            delay(100);
          }
          mb_matrix.print("TAG2 OK");
          delay(2000);
          tagid = ++tagid;                                        // 次のタグに進む
          mb_matrix.show(left_bmp);                               // Tag3 (左矢印）をLEDマトリックスに表示
          break;
        case 3:                                                   // Tag3 の学習結果を保存
          while(!huskylens.writeLearn(3)) {
            Serial.println(F("learn object Tag3 failed!")); 
            delay(100);
          }
          mb_matrix.print("TAG3 OK");
          delay(2000);
          tagid = ++tagid;                                        // タグ学習完了（4）に移行                    
          huskylens.saveModelToSDCard(1);                         // 学習モデルをSDカードに保存
          mb_matrix.print("SAVE OK");                             // 保存結果をLEDマトリックスに表示
          delay(5000);                                            // 待機
          setup_stat = 3;                                         // 実行モードに移行
          break;
      }
    }
    delay(10);
  }
  
  if (! digitalRead(buttonB)) {
    if (setup_stat == 1) {                                        // 起動後モード選択で　Bボタン押下時：実行モード
      Serial.println("Button B pressed");
      huskylens.loadModelFromSDCard(1);                           // アルゴリズムのSDカード保存モデル（1）をロード
      mb_matrix.show(B_bmp);                                      // microbit のLEDマトリックスに「B」を表示
      delay(1000);
      mb_matrix.print("RUN MODE");                                // microbit のLEDマトリックスに既存モデルロードであると表示
      setup_stat = 3;                                             // 実行モードに移行
    }
    delay(10);
  }
  
  if (setup_stat == 3) {                                          // 実行モードで処理
    findtag = 0;                                                  // 認識済みタグのカウンタを初期化（0：Tag存在なし）

    // Tag1 が認識されたかを確認
    if (!huskylens.request(1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
    else if(!huskylens.available()) {
      Serial.println(F("No block Tag1 appears on the screen!"));}
    else                                                          //　Tag1が認識された場合
    {
      HUSKYLENSResult result = huskylens.read();                  // 認識結果を取得
      printResult(result);                                        // 認識結果をシリアルコンソールへ出力
      mb_matrix.show(foward_bmp);                                 // LEDマトリックスに矢印を表示
      left = 255; right = 255;                                    // 前進指示
      findtag = 1;                                                // 認識済みタグを１にする
    }

    // Tag2 が認識されたかを確認
    if (!huskylens.request(2)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
    else if(!huskylens.available()) {
      Serial.println(F("No block Tag2 appears on the screen!"));}
    else                                                          // Tag2が認識された場合
    {
      HUSKYLENSResult result = huskylens.read();                  // 認識結果を取得
      printResult(result);                                        // 認識結果をシリアルコンソールへ出力
      mb_matrix.show(right_bmp);                                  // LEDマトリックスに矢印を表示
      left = 511; right = -511;                                   // 右転回指示
      findtag = 2;                                                // 認識積みタグを2にする
    }
    
    // Tag3 が認識されたかを確認
    if (!huskylens.request(3)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
    else if(!huskylens.available()) {
      Serial.println(F("No block Tag3 appears on the screen!"));}
    else                                                          // Tag3が認識された場合
    {
      HUSKYLENSResult result = huskylens.read();                  // 認識結果を取得
      printResult(result);                                        // 認識結果をシリアルコンソールへ出力
      mb_matrix.show(left_bmp);                                   // LEDマトリックスに矢印を表示
      left = -511; right = 511;                                   // 左転回指示
      findtag = 3;                                                // 認識積みタグを3にする
    }
    
    if (findtag == 0) { left = 255; right = 255; }                // 認識済みタグがない場合は、前進指示
    
    tamiya_robot_speed (left, right);                             // モーターへの出力を実施  
  }
}
// -------------------------------------------------------------
// HUSKYLENS認識結果のシリアルコンソールへの出力関数
// -------------------------------------------------------------
void printResult(HUSKYLENSResult result){                         
    if (result.command == COMMAND_RETURN_BLOCK){                  // バンディングボックスを認識した場合の処理
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){             // 経路（矢印）を認識した場合の処理 
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");                        // 認識不明の場合
    }
}
