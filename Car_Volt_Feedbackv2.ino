#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <INA226.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

const int buttonPin = 25;
const int buttonPin2 = A10;
const int relay = A8;
const int Lphoto = A0;
const int Rphoto = A1;
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;     
int int_left, int_right;
volatile int buttonState = LOW;
volatile int lastButtonState = HIGH;
volatile int buttonState2 = LOW;
volatile int lastButtonState2 = HIGH;
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

int Motor_PWM = 86;


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
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
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

  if (SERIAL.available())
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
  }







  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

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
        Serial.println(newV);
      }
    }
    oldV=newV;
}



//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println(F("Start"));
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  
  
  Serial.print("1");
  //Pan=PL4=>48, Tilt=PL5=>47
  servo_pan.attach(48);
  servo_tilt.attach(47);
  Serial.print("2");
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  Wire.setWireTimeout(5000,true);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  Serial.print("3");
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("AI Robot")); 
  display.display();

  Serial.print("4");

  //Setup Voltage detector
  pinMode(A0, INPUT);

  //
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(relay, OUTPUT);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets(); 
  Serial.print("5");   

  Serial.println("Initialize INA226");
  Serial.println("-----------------------------------------------");
  // Default INA226 address is 0x40
  ina.begin();
  // Configure INA226
  ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_2116US, INA226_SHUNT_CONV_TIME_2116US, INA226_MODE_SHUNT_BUS_CONT);
  // Calibrate INA226. Rshunt = 0.002 ohm, Max excepted current = 4A
  ina.calibrate(0.002, 4);

  // measure the sensors reading at ambient light intensity 
  Serial.print("calibrate photoresistor in light");
  delay(5000);        // delay 5000 ms
  int_adc0=analogRead(Lphoto);   // Left sensor at ambient light intensity
  int_adc1=analogRead(Rphoto);   // Right sensor at ambient light intensity
  Serial.print("Left : ");
  Serial.println(int_adc0);
  Serial.print("Right : ");
  Serial.println(int_adc1);
  delay(1000); 
  Serial.println("\nCalibration in progress, cover the sensors with your fingers (~ 8 sec to set)......");
  Serial.println("************ Put Fingers *****************");
  delay(5000);        // delay 5000 ms
  Serial.println("********* START Calibration **************");
  // measure the sensors reading at zero light intensity  
  int_adc0_c=analogRead(Lphoto);   // Left sensor at zero light intensity
  int_adc1_c=analogRead(Rphoto);   // Right sensor at zero light intensity
  // calculate the slope of light intensity to ADC reading equations  
  int_adc0_m=(int_adc0-int_adc0_c)/100;
  int_adc1_m=(int_adc1-int_adc1_c)/100;
  delay(10000);        // delay 10000 ms 
  Serial.println("\n******** Completed! Remove your hands ********");
  delay(4000);        // delay 4000 ms
}
  

void loop() {
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(2000);
    mpu.update();   
    Serial.print("6");
                  
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
      Serial.print(millis());
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
    Serial.print("8");
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

    Serial.print("original_angleZ");
    Serial.println(original_angleZ);
    
    while (mpu.getAngleZ() > original_angleZ - 90) {
      mpu.update();
      Serial.print("r1:");
      Serial.println(mpu.getAngleZ());
      rotate_2();
      delay(10);
      STOP();
      if (mpu.getAngleZ() <= original_angleZ - 90) {
        STOP();
        break;
      }
    }
    Serial.print("9");
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
  lastButtonState = buttonState;

  int buttonState2 = digitalRead(buttonPin2);
  if (buttonState2 == LOW && lastButtonState2 == HIGH) {
    Serial.println("get photo data:");
    delay(2000);
    int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
    int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m; 
    Serial.print("L: ");
    Serial.print(int_left);
    Serial.print(" R: ");
    Serial.println(int_right);
    while (int_left > int_right + 1){
      LEFT_2();
      Serial.print("l");
      delay(10);
      int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
      int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m;
    }
    STOP();
    delay(1000);
    while (int_right > int_left + 1){
      RIGHT_2();
      Serial.print("r");
      delay(10);
      int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
      int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m;
    }
    STOP();
    delay(1000);
    float lastpower = 0;
    digitalWrite(relay, LOW);
    float Voc = ina.readBusVoltage();
    float Isc = 0; 
    float power = 0;
    while (power >= lastpower){
      lastpower = power;
      digitalWrite(relay, LOW);
      Voc = ina.readBusVoltage();
      Serial.print("volt: ");
      Serial.print(Voc);
      digitalWrite(relay, HIGH);
      Isc = ina.readShuntCurrent();
      Serial.print(" current: ");
      Serial.print(Isc);
      power = Voc * Isc;
      Serial.print(" power: ");
      Serial.println(power);
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(power);
      display.println(" W");
      display.display();
      ADVANCE();
      delay(20);
      STOP();
    }
    BACK(MIN_PWM,MIN_PWM,MIN_PWM,MIN_PWM);
    delay(10);
    STOP();
    digitalWrite(relay, LOW);
    Voc = ina.readBusVoltage();
    digitalWrite(relay, HIGH);
    Isc = ina.readShuntCurrent();
    power = Voc * Isc;
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(power);
  }
  lastButtonState2 = buttonState2;
    
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
    UART_Control(); //get USB and BT serial data

    //constrain the servo movement
    //pan = constrain(pan, servo_min, servo_max);
    //tilt = constrain(tilt, servo_min, servo_max);
    
    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);
  }
  if (voltCount>=5){
    voltCount=0;
    //sendVolt();
  }
  
  
}
