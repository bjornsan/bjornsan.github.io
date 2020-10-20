#include <RH_ASK.h>
#include <SPI.h>

// left engine
#define enB 5
#define motorTwoCCW 6
#define motorTwoCW 7

// right engine
#define enA 11
#define motorOneCW 9
#define motorOneCCW 8

int motorSpeedA = 0;
int motorSpeedB = 0;

// RF-COMM - 433Mhz module 
RH_ASK driver(2000, 4, 12);

// Joystick values
int sw_state = LOW;

// Needed to convert to 8bit format for transmitting (0 - 255)
uint8_t recivedX;
uint8_t recivedY;

// Recieved values remapped to 10bit values (0 - 1023)
int joyX_10bit;
int joyY_10bit;

void setup() {
  Serial.begin(9600);

  // comm init
  if (!driver.init()) {
    Serial.println("init failed");
  }

  // engine setup
  pinMode(motorOneCW, OUTPUT);
  pinMode(motorOneCCW, OUTPUT);
  pinMode(motorTwoCW, OUTPUT);
  pinMode(motorTwoCCW, OUTPUT);

}

void loop() {

  getJoystickValues();
  moveRobot();
  delay(300);

}

void getJoystickValues() {
  
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  
  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    recivedX = buf[0];
    recivedY = buf[1];

    joyX_10bit = map(joyValX, 0, 255, 0, 1023);
    valY_10bit = map(joyValY, 0, 255, 0, 1023);
    
    // Message with a good checksum received, dump it.
    driver.printBuffer("Got:", buf, buflen);
  }
}

void moveRobot() {
  
  // Y-axis used for forward and backward control
  if (joyY_10bit < 470) {
    motorSpeedA = joyY_10bit;
    motorSpeedB = motorSpeedA;
    backward();
  }
  else if (joyY_10bit > 550) {
    motorSpeedA = joyY_10bit;
    motorSpeedB = motorSpeedA;
    forward();
  }
  else {
    stopEngine();
  }
  
  // X-axis used for left and right control
  if (joyX_10bit < 470) {
    turnLeft();
  }
  if (joyX_10bit > 550) {
    turnRight();
  }

  // stop engines from buzzing at low speeds
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
}

void forward() {
  digitalWrite(motorOneCW, LOW);
  digitalWrite(motorOneCCW, HIGH);

  digitalWrite(motorTwoCW, LOW);
  digitalWrite(motorTwoCCW, HIGH);

  motorSpeedA = map(joyY_10bit, 550, 1023, 0, 255);
  motorSpeedB = map(joyY_10bit, 550, 1023, 0, 255);

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}

void backward() {
  digitalWrite(motorOneCW, HIGH);
  digitalWrite(motorOneCCW, LOW);

  digitalWrite(motorTwoCW, HIGH);
  digitalWrite(motorTwoCCW, LOW);

  motorSpeedA = map(joyY_10bit, 470, 0, 0, 255);
  motorSpeedB = map(joyY_10bit, 470, 0, 0, 255);

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}

void turnLeft() {
  
    int x_map = map(joyX_10bit, 470, 0, 0, 255);
    motorSpeedA = motorSpeedA - x_map;
    motorSpeedB = motorSpeedB + x_map;
      
   if (motorSpeedA < 0) {
     motorSpeedA = 0;
   }
   if (motorSpeedB > 255) {
      motorSpeedB = 255;
   }
   analogWrite(enA, motorSpeedA);
   analogWrite(enB, motorSpeedB);
}

void turnRight() {
  
  int x_map = map(joyX_10bit, 550, 1023, 0, 255);
  motorSpeedA = motorSpeedA + x_map;
  motorSpeedB = motorSpeedB - x_map;
  // Confine the range from 0 to 255
  if (motorSpeedA > 255) {
    motorSpeedA = 255;
  }
  if (motorSpeedB < 0) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}

void stopEngine() {
  
  motorSpeedA = 0;
  motorSpeedB = 0;
  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);
}
