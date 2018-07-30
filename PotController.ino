#define thermo A0
#define heaterH 8
#define heaterL 9
#define motor 10
#define thermo2 A2
#define RotEncA 3
#define RotEncB 4

#define VERSION "0.0.0.1"

#include <I2CLiquidCrystal.h>
#include <Wire.h>
#define resetPin 11
I2CLiquidCrystal lcd(20, true);
                  //  |    +--- set true if the power suply is 5V, false if it is 3.3V
                  //  +-------- contrast (0-63)

volatile int RotValue = 0;
bool isChange = false;


int sensorValue = 0;
int sensorValue2 = 0;
int targetValue = 0;
int stateH = 0;
int stateL = 0;
int mP = 0;
int mI = 0;
int mD = 0;

int cycleTime = 5;
int duty = 30;

long previousMillis = 0;

//起動時間用
unsigned long utime=0;

enum MenuList { TopMenue, ShowMenue, InputTargetValue, RealTime, InputHeaterState, Initialize} list;


void chkRotEnc1(void){
  int A = digitalRead(RotEncA);
  int B = digitalRead(RotEncB);

  if(A){
    if(B){
      //L (3)
      RotValue--;
    }else{
      //R (1)
      RotValue++;
    }
  }else{
    if(B){
      //R (2)
      RotValue++;
    }else{
      //L (4)
      RotValue--;
    }
  }
  isChange = true;
}


void setup() {
	//IC_Clock_ES1でデバッグ用
	digitalWrite(4,LOW);
	pinMode(4,OUTPUT);

  Serial.begin(9600);

  //RotEnc
  pinMode(RotEncA, INPUT);
  pinMode(RotEncB, INPUT);
  digitalWrite(RotEncA, HIGH);
  digitalWrite(RotEncB, HIGH);
  attachInterrupt(1, chkRotEnc1, CHANGE);

  // set up the LCD's number of columns and rows: 
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  delay(1);
  digitalWrite(resetPin, HIGH);
  delay(10);
  lcd.begin(16, 2);
  lcd.print("Hello World");

  //Heater Initialize
  digitalWrite(heaterH,LOW);
  digitalWrite(heaterL,LOW);
  digitalWrite(motor,LOW);
  pinMode(heaterH,OUTPUT);
  pinMode(heaterL,OUTPUT);
  pinMode(motor,OUTPUT);
 
  list = Initialize;
  char tmp[10]={""};
  serialCUI(tmp);

//test = 1;
}



void serialCUI(char serialData[]){
	
	switch( list ){
	case TopMenue:
		if(serialData[0] == '1'){
			//1:Set Target Value
                        Serial.print("Before Target Value:");
			Serial.println(targetValue);
			Serial.print("new Target Value = ");
			list = InputTargetValue;
		}else if(serialData[0] == '2'){
                        //2:View PID Parameter
                        char tmp[100];
                        sprintf(tmp, "P=%d  I=%d  D=%d", mP, mI, mD);
                        Serial.println(tmp);
                        list = ShowMenue;
                        break;
		}else if(serialData[0] == '3'){
                        //3:View realtime Value
                        list = RealTime;
                        break;
		}else if(serialData[0] == '4'){
                        //4:Set Heater Status
                        Serial.print("HeaterH=");
                        Serial.print(heaterH);
                        Serial.print("HeaterL=");
                        Serial.println(heaterL);
                        Serial.println("Input H=ON/OFF L=ON/OFF  M=ON");
                        list = InputHeaterState;
                        break;
                }else{
			Serial.println("bad Input");
			list = ShowMenue;
		}
		break;
        case InputHeaterState:
                heaterState(serialData);
                showMenue();
                break;
	case InputTargetValue:{
                int tmpValue =0, tmpSerial=0;
                for(int i=0; serialData[i] != NULL; i++){
                  tmpSerial = serialData[i] - '0';
                  if(0 <= tmpSerial && tmpSerial <=9){
                    tmpValue = tmpValue * 10 + tmpSerial;
                  }else{
                    Serial.println("error data");
                    showMenue();
                  }
                }
                 
                targetValue = tmpValue;
                list = TopMenue;
                Serial.print("new Target Value=");
                Serial.println(targetValue);
                showMenue();
             }break;
	case RealTime:
                if(serialData[0]!= NULL){
                  list = TopMenue;
                  showMenue();
                }
                showValue();
		break;
	case Initialize:
	default:
		Serial.print("SensorPot Ver=");
		Serial.println(VERSION);
//		Serial.println("0.0.0.1");
		Serial.print("Now Value:");
		Serial.println(sensorValue);
		Serial.print("Target Value:");
		Serial.println(targetValue);
		Serial.print("stateH:");
		Serial.print(stateH);
		Serial.print(" stateL");
		Serial.println(stateL);
		Serial.print("P:");
		Serial.print(mP);
		Serial.print(" I:");
		Serial.print(mI);
		Serial.print(" D:");
		Serial.println(mD);
	//Menue
	case ShowMenue:
                showMenue();
		break;
	}
	
}

void showValue(){
  char tmp[100];
  sprintf(tmp, "time:, %ld, Now, %d, target, %d, PID, %d, %d, %d, H, %d, L, %d",
    utime, sensorValue, targetValue, mP, mI, mD, stateH, stateL);
  Serial.println(tmp);
}

void showMenue(){
  Serial.println(" ");
  Serial.println("1:Set Target Value");
  Serial.println("2:View PID Parameter");
  Serial.println("3:View realtime Value");
  Serial.println("4:Set Heter Status");
  list = TopMenue;
}

void heaterState(char tmp[]){
  if(tmp[0] != NULL){
    if(strcmp(tmp,"H=ON") == 0){
      Serial.println("Set HeaterH ON");
      stateH = 1;
    }else if(strcmp(tmp,"H=OFF") == 0){
      Serial.println("Set HeaterH OFF");
      stateH = 0;
    }else if(strcmp(tmp,"L=ON") == 0){
      Serial.println("Set HeaterL ON");
      stateL = 1;
    }else if(strcmp(tmp,"L=OFF") == 0){
      Serial.println("Set HeaterL OFF");
      stateL = 0;
    }else if(strcmp(tmp,"M=ON") == 0){
      Serial.println("Motor Drive!!");
      digitalWrite(motor,HIGH);
      delay(10000);
      digitalWrite(motor,LOW);
      Serial.println("Motor Done!!");
    }else{
      Serial.println("input error");
    }
  }
}

void drawLCD(){
  char tmp[20];
  char tmp2[20];
  sprintf(tmp, "t:%ld Now:%d   ", utime, sensorValue);
  sprintf(tmp2, "H:%d L:%d Set:%d   ", stateH, stateL, targetValue);
//  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(tmp);
  lcd.setCursor(0,1);
  lcd.print(tmp2);
}

void loop() {
  utime = millis()/1000;

  char tmp[10] = "";

	//シリアルが来ていたらメニューを出す
	for(int i=0;Serial.available() > 0;i++){
		tmp[i] = Serial.read();
		if(i==9 || Serial.available() < 1){
			Serial.flush();	//キャッシュクリア
			serialCUI(tmp);
		}
	}
	//リアルタイム表示モードなら表示する
	if(list == RealTime) serialCUI("");
  
  
  if(isChange){
    drawLCD();
    targetValue += RotValue * RotValue * RotValue;
    RotValue = 0;
    isChange = false;
  }
  
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > 1000){
    previousMillis = currentMillis;

    //LCDに表示をする
    drawLCD();
  
    int PID_Output = 0;
  
    sensorValue2 = sensorValue;
    sensorValue=0;
    sensorValue += analogRead(thermo);
    sensorValue += analogRead(thermo);
    sensorValue += analogRead(thermo);
    sensorValue += analogRead(thermo);
    sensorValue += analogRead(thermo);
    sensorValue /= 5;
  
  //  dxdt2 = dxdt;
  //  dxdt = sensorValue2 - sensorValue;
  
    heate();  //通常運転モード
  //  dutyHeate(4);    //デューティーモード(試験用)
    //updateHeater
    digitalWrite(heaterH,stateH);
    digitalWrite(heaterL,stateL);
  }
}

int dutyHeate(int duty){
  if(utime % 10 < duty){
    stateH = HIGH;
    stateL = LOW;
  }else{
    stateH = LOW;
    stateL = LOW;
  }
}

int heate(){
  int dx = sensorValue - targetValue;
  if(sensorValue < targetValue){
    //あつすぎる ヒーターOFF
    stateH = LOW;
    stateL = LOW;
  }else{
    stateH = HIGH;
  }

}

//int maxdxdt = 0;
//unsigned long onTimeH =0;
//
//int heate(){
//  int dx = targetValue - sensorValue;  
//  if(dx < 0) return -1;
//  
//  //目標までとても離れている
//  if(1000 < dx){
//    if(heaterH == 0){
//      heaterH = 1;
//      maxdxdt = 0;
//      onTimeH = utime;
//    }else{
//      
//    }
//  }else{
//  //目標までそんなに離れていない
//  }
//}
