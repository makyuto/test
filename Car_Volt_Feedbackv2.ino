#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <INA226.h>
//#include <Servo.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
SoftwareSerial mySerial(4, 6); // RX, TX
float Isc = 0; 
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
//Servo servo_pan;
//Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
int encoderValue = 0;
int message_list [2];

//pins and vars
const int buttonPin = 25;
const int buttonPin2 = A10;
const int buttonPin3 = 50;
const int relay = A11;
const int Lphoto = A6;
const int Rphoto = A7;
const int IRsensor = A8;
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;     
int int_right, int_left;
volatile int buttonState = LOW;
volatile int lastButtonState = HIGH;
volatile int buttonState2 = LOW;
volatile int lastButtonState2 = HIGH;
volatile int buttonState3 = LOW;
volatile int lastButtonState3 = HIGH;
#define echoPin_F 6
#define trigPin_F 7
#define echoPin_B 47
#define trigPin_B 48
#define echoPin_L 40
#define trigPin_L 41
#define echoPin_R 32
#define trigPin_R 33
#define wlsensor A0


//MPU
MPU6050 mpu(Wire);
INA226 ina;


unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction
#define Encoder_A 18
#define Encoder_B 31

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 70;


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM + 10); //fix error
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM + 10);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */
  /* if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }
    */

    //BT Control
    /*
      Receive data from app and translate it to motor movements
    */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
    {
      BT_Data = Serial3.read();
      Serial3.flush();
      BT_alive_cnt = 100;
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("BT_Data = ");
      display.println(BT_Data);
      display.display();
    }
  //}

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}

/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0);
    if(newV!=oldV) {
      if (!Serial3.available()) {
        Serial3.println(newV);
      }
    }
    oldV=newV;
}

//task1
void task_1() {
    delay(2000);
    mpu.update(); 
                  
    int start_t= millis();

    int original_angleZ = mpu.getAngleZ();
    int count = 0;
    while(count < 200) {
      ADVANCE();
      delay(10);
      count++;
      STOP();
      mpu.update();
      while (mpu.getAngleZ() > original_angleZ + 3){
        mpu.update();
        rotate_2();
        delay(10);
        STOP();
      }
      while (mpu.getAngleZ() < original_angleZ - 3) {
        mpu.update();
        rotate_1();
        delay(10);
        STOP();
      }
    }

    
    STOP();
    mpu.update();
    while (mpu.getAngleZ() > original_angleZ + 5){
        mpu.update();
        rotate_2();
        delay(10);
        STOP();
      }
      while (mpu.getAngleZ() < original_angleZ - 5) {
        mpu.update();
        rotate_1();
        delay(10);
        STOP();
      }
    delay(2000);

    mpu.update();
    original_angleZ = mpu.getAngleZ();
    count = 0;
    while(count < 200) {
      BACK(MIN_PWM,MIN_PWM,MIN_PWM,MIN_PWM);
      delay(10);
      count++;
      STOP();
      mpu.update();
      while (mpu.getAngleZ() > original_angleZ + 3){
        mpu.update();
        rotate_2();
        delay(10);
        STOP();
      }
      while (mpu.getAngleZ() < original_angleZ - 3) {
        mpu.update();
        rotate_1();
        delay(10);
        STOP();
      }
    }
    STOP();
    mpu.update();
    while (mpu.getAngleZ() > original_angleZ + 5){
      mpu.update();
      rotate_2();
      delay(10);
      STOP();
    }
    while (mpu.getAngleZ() < original_angleZ - 5) {
      mpu.update();
      rotate_1();
      delay(10);
      STOP();
    }
    delay(2000);

    mpu.update();
    int original_angleX = mpu.getAngleX();
    int original_angleY = mpu.getAngleY();
    original_angleZ = mpu.getAngleZ();

    
    while (mpu.getAngleZ() > original_angleZ - 90) {
      mpu.update();
      rotate_2();
      delay(10);
      STOP();
      if (mpu.getAngleZ() <= original_angleZ - 90) {
        STOP();
        break;
      }
    }
    STOP();
    delay(2000);

    rotate_2();
    delay(3200);
    STOP();
    delay(2000);

    mpu.update();
    original_angleX = mpu.getAngleX();
    original_angleY = mpu.getAngleY();
    original_angleZ = mpu.getAngleZ();
    while (mpu.getAngleZ() < original_angleZ + 90) {
      mpu.update();
      rotate_1();
      delay(10);
      STOP();
      if (mpu.getAngleZ() >= original_angleZ + 90) {
        STOP();
        break;
      }
    }
    STOP();
    delay(2000);

    rotate_1();
    delay(8000);
    STOP();
    delay(2000);
}

//task2
void task_2() {
    Serial.println("get photo data:");
    delay(2000);
    int_left=(analogRead(Lphoto)-int_adc0_c)/int_adc0_m;
    int_right=(analogRead(Rphoto)-int_adc1_c)/int_adc1_m; 
    Serial.print("R : ");
    Serial.print(int_right);
    Serial.print(" L: ");
    Serial.println(int_left);
    float lastV = 0;
    digitalWrite(relay, LOW);
    float Voc = ina.readBusVoltage();
    
    float power = 0;
    while (Voc > lastV - 0.01){
      lastV = Voc;
      digitalWrite(relay, HIGH);
      Voc = ina.readBusVoltage();
      Serial.print("volt: ");
      Serial.println(Voc);
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(Voc);
      display.println(" V");
      display.display();
      ADVANCE();
      delay(10);
    }
    STOP();
    delay(100);
    BACK(MIN_PWM,MIN_PWM,MIN_PWM,MIN_PWM);
    delay(700);
    STOP();
    delay(1000);

    int_left=(analogRead(Lphoto)-int_adc0_c)/int_adc0_m;
    int_right=(analogRead(Rphoto)-int_adc1_c)/int_adc1_m;
    digitalWrite(relay, HIGH);
    Voc = ina.readBusVoltage();
    
    //if left brighter
    if((int_right > int_left)){ 
      rotate_2();
      delay(500);
    }
    STOP();
    delay(1000);
    //if right brighter
    if ((int_left > int_right)){
      rotate_1();
      delay(500);
    }

    STOP();
    delay(2000);

    digitalWrite(relay, HIGH);
    delay(2000);
    Voc = ina.readBusVoltage();
    Serial.print(Voc);
    
    digitalWrite(relay, LOW);
    delay(2000);
    Isc = ina.readShuntCurrent();
    Serial.print(Isc);
    
    digitalWrite(relay, LOW);
    power = Voc * Isc;
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Power: ");
    display.print(power);
    display.println(" W");
    display.display();
    Serial.println(power);
    digitalWrite(relay, HIGH);
}

void displayMessage(String x){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(x);
  display.display();
}

void Encoder_subroutine() {
  //check the voltage of another encoder pin
  if (digitalRead(Encoder_B)) {
    encoderValue--;
  }
  else {
    encoderValue++;
  }
}

//ps2 mouse set
unsigned long millisStart;
long MouseX = 0;
long MouseY = 0;
char stat,x,y;

int posX = 2; //starting coordinate
int posY = 2;

byte PS2ReadByte = 0;

#define PS2CLOCK  10
#define PS2DATA   11

void PS2GoHi(int pin){
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void PS2GoLo(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void PS2Write(unsigned char data){
  unsigned char parity=1;

  PS2GoHi(PS2DATA);
  PS2GoHi(PS2CLOCK);
  delayMicroseconds(300);
  PS2GoLo(PS2CLOCK);
  delayMicroseconds(300);
  PS2GoLo(PS2DATA);
  delayMicroseconds(10);
  PS2GoHi(PS2CLOCK);

  while(digitalRead(PS2CLOCK)==HIGH);

  for(int i=0; i<8; i++){
    if(data&0x01) PS2GoHi(PS2DATA);
    else PS2GoLo(PS2DATA);
    while(digitalRead(PS2CLOCK)==LOW);
    while(digitalRead(PS2CLOCK)==HIGH);
    parity^=(data&0x01);
    data=data>>1;
  }

  if(parity) PS2GoHi(PS2DATA);
  else PS2GoLo(PS2DATA);

  while(digitalRead(PS2CLOCK)==LOW);
  while(digitalRead(PS2CLOCK)==HIGH);

  PS2GoHi(PS2DATA);
  delayMicroseconds(50);

  while(digitalRead(PS2CLOCK)==HIGH);
  while((digitalRead(PS2CLOCK)==LOW)||(digitalRead(PS2DATA)==LOW));

  PS2GoLo(PS2CLOCK);
}

unsigned char PS2Read(void){
  unsigned char data=0, bit=1;

  PS2GoHi(PS2CLOCK);
  PS2GoHi(PS2DATA);
  delayMicroseconds(50);
  while(digitalRead(PS2CLOCK)==HIGH);

  delayMicroseconds(5);
  while(digitalRead(PS2CLOCK)==LOW);

  for(int i=0; i<8; i++){
    while(digitalRead(PS2CLOCK)==HIGH);
    if(digitalRead(PS2DATA)==HIGH) data|=bit;
    while(digitalRead(PS2CLOCK)==LOW);
    bit=bit<<1;
  }

  while(digitalRead(PS2CLOCK)==HIGH);
  while(digitalRead(PS2CLOCK)==LOW);
  while(digitalRead(PS2CLOCK)==HIGH);
  while(digitalRead(PS2CLOCK)==LOW);

  PS2GoLo(PS2CLOCK);

  return data;
}

void PS2MouseInit(void){
  PS2Write(0xFF);
  for(int i=0; i<3; i++) PS2Read();
  PS2Write(0xF0);
  PS2Read();
  delayMicroseconds(100);
}

void PS2MousePos(char &stat, char &x, char &y){
  PS2Write(0xEB);
  PS2Read();
  stat=PS2Read();
  x=PS2Read();
  y=PS2Read();
}

void spray() {
  digitalWrite(relay,LOW); // on
  delay(1000);
  digitalWrite(relay,HIGH);
  delay(500);
  digitalWrite(relay,LOW); //off
  delay(2000);
  digitalWrite(relay,HIGH);
}

long measure_distance(char dir) {
  long duration;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  if (dir == 'F'){
  digitalWrite(trigPin_F, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin_F, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin_F, LOW);
  duration = pulseIn(echoPin_F, HIGH);
  }
  else if (dir == 'B') {
  digitalWrite(trigPin_B, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin_B, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin_B, LOW);
  duration = pulseIn(echoPin_B, HIGH);
  }
  else if (dir == 'L') {
  digitalWrite(trigPin_L, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin_L, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin_L, LOW);
  duration = pulseIn(echoPin_L, HIGH);
  }
  else if (dir == 'R') {
  digitalWrite(trigPin_R, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin_R, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin_R, LOW);
  duration = pulseIn(echoPin_R, HIGH);
  }
  long distance_in_cm = (duration/2.0) / 29.1;
  return distance_in_cm;
}

int state[] = {0,0,1,posX,posY};

void sendtopi() {
  state[3] = posX;
  state[4] = posY;
 
}


void block_warning(){
  state[1] = 1;
  sendtopi();
}

void waterlevelcheck(){
  if (analogRead(wlsensor) < 600){
    
  }
}

void ADVANCE_2(int d, int check){ 
  // distance = x * 10cm
  STOP();
  delay(200);
  mpu.update();
  float original_angleZ = mpu.getAngleZ();
  MouseY = 0; //62.5 = 1cm
  MouseX = 0;
  while(MouseY <  625 * d) {
    if(millis() < millisStart){
        millisStart = millis();
    }
    if(millis() - millisStart > 100){
      displayMessage(String(posX)+","+String(posY)+"\n"+String(MouseX)+String(MouseY));
      PS2MousePos(stat,x,y);
      MouseX += x;
      MouseY += y;
      if (abs(original_angleZ/360) < 5){
        posY = 2 + MouseY/625 ; 
      }
      else if (abs(original_angleZ/360 - 90) < 5){
        posX = 2 - MouseY/625 ; 
      }
      else if (abs(original_angleZ/360 + 90) < 5){
        posX = 2 + MouseY/625 ;
      }
      else if (abs(original_angleZ/360 - 180) < 5){
        posY = 2 - MouseY/625 ; 
      }
      sendtopi();
      
      if (measure_distance('F') <= 25 || measure_distance('F') == 1200){
        STOP();
        delay(5000);
        if (measure_distance('F') <= 25 || measure_distance('F') == 1200){
          block_warning();
        }
      }
      else {
        state[1] = 0;
        ADVANCE();
        delay(50);
        STOP();
        mpu.update();
        while (mpu.getAngleZ() > original_angleZ + 2){
          mpu.update();
          rotate_2();
          delay(20);
          STOP();
        }
        while (mpu.getAngleZ() < original_angleZ - 2) {
          mpu.update();
          rotate_1();
          delay(20);
          STOP();
        }
      }
      millisStart = millis();
      STOP();
    }
  }
  STOP();
  delay(50);
  if (check > 0){
      while (MouseX >  200){ 
        LEFT_2();
        delay(20);
        STOP();
      }
      while (MouseX >  200){ 
        LEFT_2();
        delay(20);
        STOP();
      }
      while (mpu.getAngleZ() > original_angleZ){
        mpu.update();
        rotate_2();
        delay(10);
        STOP();
      }
      while (mpu.getAngleZ() < original_angleZ){
        mpu.update();
        rotate_1();
        delay(10);
        STOP();
      }
  }
  STOP();
  delay(100);
}


void ROTATE_L (){
  mpu.update();
  int original_angleZ = mpu.getAngleZ();
  while (mpu.getAngleZ() > original_angleZ - 90) {
      mpu.update();
      STOP();
      if (measure_distance('L') <= 25 || measure_distance('L') == 1200){
        STOP();
        delay(5000);
        if (measure_distance('L') <= 25 || measure_distance('L') == 1200){
          block_warning();
        }
      }
      else {
        state[1] = 0;
        rotate_2();
        delay(50);
      }
      if (mpu.getAngleZ() <= original_angleZ - 90) {
        STOP();
        break;
      }
    }
    STOP();
}

void ROTATE_R (){
  mpu.update();
  int original_angleZ = mpu.getAngleZ();
  while (mpu.getAngleZ() < original_angleZ + 90) {
      mpu.update();
      STOP();
      if (measure_distance('R') <= 25 || measure_distance('R') == 1200){
        STOP();
        delay(5000);
        if (measure_distance('R') <= 25 || measure_distance('R') == 1200){
          block_warning();
        }
      }
      else {
        state[1] = 0;
        rotate_1();
        delay(50);
      }
      if (mpu.getAngleZ() >= original_angleZ + 90) {
        STOP();
        break;
      }
    }
    STOP();
}

void cleanall(){
  
}


//Where the program starts
void setup()
{
  
  SERIAL.begin(115200); // USB serial setup
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup

    
  
  //Pan=PL4=>48, Tilt=PL5=>47
  //servo_pan.attach(48);
  //servo_tilt.attach(47);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  Wire.setWireTimeout(5000,true);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot"); 
  display.display();

  //Setup Voltage detector
  pinMode(A0, INPUT);

  //
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(Lphoto, INPUT);
  pinMode(Rphoto, INPUT);
  
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets(); 

  // Default INA226 address is 0x40
  ina.begin();
  // Configure INA226
  ina.configure(INA226_AVERAGES_64, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
  // Calibrate INA226. Rshunt = 0.002 ohm, Max excepted current = 4A
  ina.calibrate(0.002, 4);

  // measure the sensors reading at ambient light intensity 
  /*
  Serial.print("calibrate photoresistor in light");
  //delay(1000);        
  int_adc0=analogRead(Lphoto);   // Left sensor at ambient light intensity
  int_adc1=analogRead(Rphoto);   // Right sensor at ambient light intensity
  Serial.print("Left : ");
  Serial.println(int_adc0);
  Serial.print("Right : ");
  Serial.println(int_adc1);
  Serial.println("\nCalibration in progress, cover the sensors with your fingers (~ 8 sec to set)......");
  Serial.println("************ Put Fingers *****************");
  //delay(1000); 
  Serial.println("********* START Calibration **************");
  // measure the sensors reading at zero light intensity  
  int_adc0_c=analogRead(Lphoto);   // Left sensor at zero light intensity
  int_adc1_c=analogRead(Rphoto);   // Right sensor at zero light intensity
  // calculate the slope of light intensity to ADC reading equations  
  int_adc0_m=(int_adc0-int_adc0_c)/100;
  int_adc1_m=(int_adc1-int_adc1_c)/100;
  Serial.println("\n******** Completed! Remove your hands ********");
  attachInterrupt(digitalPinToInterrupt(Encoder_A), Encoder_subroutine, FALLING);
  */
  
  //ps2mouse setup
  PS2GoHi(PS2CLOCK);
  PS2GoHi(PS2DATA);
  Serial.begin(115200);
  while(!Serial); 
  PS2MouseInit();
  millisStart=millis();
  MouseX = 0;
  MouseY = 0;
  
  // ultrasonic 
  pinMode(echoPin_F, INPUT);
  pinMode(trigPin_F, OUTPUT);
  pinMode(echoPin_B, INPUT);
  pinMode(trigPin_B, OUTPUT);
  pinMode(echoPin_L, INPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(echoPin_R, INPUT);
  pinMode(trigPin_R, OUTPUT);
}

int c = 0;


void loop() {
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    task_1();
  }
  lastButtonState = buttonState;

  int buttonState2 = digitalRead(buttonPin2);
  if (buttonState2 == LOW && lastButtonState2 == HIGH) {
    task_2();
  }
  lastButtonState2 = buttonState2;
  
  int dest_x;
  int dest_y;

  while (!Serial.available()){
    int buttonState3 = digitalRead(buttonPin3);
    if (buttonState3 == LOW && lastButtonState3 == HIGH) {
      cleanall();
    }
  }
  if (Serial.available() > 0){
    String recv = Serial.readString();
    dest_x = (recv[0]-'0')*10 + (recv[1]-'0');
    dest_y = (recv[2]-'0')*10 + (recv[3]-'0');
    state[2] = 0;
  }
  ADVANCE_2(dest_y - 4, 0);
  ROTATE_L();
  ADVANCE_2(dest_x, 0);
  state[0] = 1; 
  sendtopi();
  time = millis();
  while(millis() > time + 10000){
    if (digitalRead(IRsensor) == LOW){
      spray();
    }
  }
  delay(500);
  ROTATE_R();
  ROTATE_R();
  state[0] = 0;

  ADVANCE_2(dest_x, 0);
  ROTATE_R();
  ADVANCE_2(dest_y - 4, 0);
  state[3] = 1;
  sendtopi();
   
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
    
    UART_Control(); //get USB and BT serial data

    //constrain the servo movement
    //pan = constrain(pan, servo_min, servo_max);
    //tilt = constrain(tilt, servo_min, servo_max);
    
    //send signal to servo
    //servo_pan.write(pan);
    //servo_tilt.write(tilt);
  }
  if (voltCount>=5){
    voltCount=0;
    //sendVolt();
  }
  
}
