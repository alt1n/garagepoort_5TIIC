// DC MOTOR GARAGE POORT MET IR/BLUETOOTH/LCD met I2C door Altin en Kenji (Alles werkt)

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27,16,2);

const int controlPin1 = 4; // connected to pin 7 on the H-bridge
const int controlPin2 = 5; // connected to pin 2 on the H-bridge
const int enablePin = 9;   // connected to pin 1 on the H-bridge

const int hallPin = 3;
const int button1Pin = 7;
const int IRPin = 2;

int motorEnabled = 0; // Turns the motor on/off
int motorSpeed = 0; // speed of the motor
int motorDirection = 1; // current direction of the motor

int hallState = 0;
int button1State = 0;
int IRstate = 0;

char inputByte;

int state = HIGH;      
int reading;           
int previous = LOW;    
long time = 0;         
long debounce = 200;

void setup() {
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(button1Pin, INPUT_PULLUP);
  digitalWrite(enablePin, LOW);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(hallPin), hallStop, FALLING);
  attachInterrupt(digitalPinToInterrupt(IRPin), IRStop, RISING);
  lcd.init();
  lcd.backlight();
}

void loop() {

  lcd.setCursor(1, 0);
  lcd.print("Button 1 state: ");
  lcd.setCursor(10, 0);
  lcd.print(state);
  lcd.setCursor(1, 1);
  lcd.print("InfraRood state: ");
  lcd.setCursor(4, 1);
  lcd.print(IRstate);
  
  delay(100);
  digitalWrite(enablePin, motorEnabled);
  IRstate = digitalRead(IRPin);
  reading = digitalRead(button1Pin);
  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == HIGH)
      state = LOW;
    else
      state = HIGH;
      motorEnabled = HIGH;
    time = millis();
  }
  previous = reading;
    Serial.print("Button 1: ");
    Serial.println(state);
    Serial.print("IR: ");
    Serial.println(IRstate);

    if (state == HIGH){
    digitalWrite(controlPin1, HIGH);
    digitalWrite(controlPin2, LOW);
    }
    if (state == LOW){
    digitalWrite(controlPin1, LOW);
    digitalWrite(controlPin2, HIGH);
    }

    while(Serial.available()>0){
      inputByte= Serial.read();
      Serial.println(inputByte);
      if (inputByte == 'U'){
        motorEnabled = LOW;
      }
      else if (inputByte == 'A'){
        motorEnabled = HIGH;
      } 
      else if (inputByte == 'L'){
        state = LOW;
      } 
      else if (inputByte == 'R'){
        state = HIGH;
      } 
    }
}

void hallStop() {
  Serial.println("STOP! STOP! STOP! STOP! STOP! STOP! STOP! STOP! STOP! STOP! ");
   motorEnabled = LOW;
  }
  void IRStop() {
  Serial.println("STOP! STOP! STOP! STOP! STOP! STOP! STOP! STOP! STOP! STOP! ");
   motorEnabled = LOW;
   state != state;
  }
  

//001
