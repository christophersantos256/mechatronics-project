#include <LiquidCrystal.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>

const int rs = 25, en = 24, d4 = 29, d5 = 28, d6 = 31, d7 = 30;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int ussensor = 41;
int ustrig = 40;
float duration, dist;
Servo sensorServo;
Servo flagServo;
Servo lockServo;
bool isCentered = false;

//flag
bool flagHeld = false;
unsigned long flagHoldStart = 0;
const unsigned long flagHoldDur = 2500;

//accelerometer
const uint8_t MPU_ADDR = 0x68;
int16_t rawAx, rawAy, rawAz;
float ax_ms2, ay_ms2, az_ms2;
float dynAx, dynAy, dynAz;
float accXY;
float lastDynAx = 0, lastDynAy = 0, lastDynAz = 0;
float filtAx = 0, filtAy = 0, filtAz = 0;
const float g = 9.81f;
const float scale = 16384.0f;
const float alpha = 0.98f;
const unsigned long accelInterval = 30;
unsigned long lastAccelTime = 0;

//doScan
const unsigned long sweepInterval = 5;
unsigned long lastSweepTime = 0;
const int sweepStep = 3;
const unsigned long pauseDuration = 1000;
int scanPhase = 0;
int sweepPos = 90;
int sweepDir = +1;
float distR=0, distL=0;
unsigned long lastTime = 0;
unsigned long phaseStart;

const unsigned long distInterval = 500;
unsigned long lastDistTime = 0;

//IR
const int irPin = 4;
bool irPaused = false;
unsigned long irPauseStart = 0;
const unsigned long irPauseDur = 10000;
bool locked = false;
bool lockEnabled = true;
const int irCountThresh   = 20;    
const unsigned long irWin  = 1000;
unsigned long irTimes[irCountThresh] = {0};  
bool prevIr = false;


//LED and Buzzer
const int ledPin = 11;
const int buzzPin = 10;
float lastAcc = 0.0f;
const float decelThresh = 1.0f;
bool alerting = false;
bool ledState = false;
const unsigned long toggleInterval = 50;
float lastAccMag = 0;
int readHorn;
int horn;

//RFID
#define RST_PIN 9
#define SS_PIN 53
MFRC522 rfid(SS_PIN, RST_PIN);

//Serial Comm
const int rxPin = 0;
const int txPin = 1; 

//------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  //US Sensor
  pinMode(ustrig, OUTPUT);
  pinMode(ussensor, INPUT);
  //Servo & LCD
  sensorServo.attach(2);
  flagServo.attach(13);
  lcd.begin(16, 2);

  //accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  //RFID
  pinMode(SS_PIN, OUTPUT);
  SPI.begin();
  rfid.PCD_Init();
  Serial.println(F("RFID init complete"));  

  //IR
  pinMode(irPin, INPUT);


  //LED, Buzzer
  pinMode(ledPin, OUTPUT);
  pinMode(buzzPin, OUTPUT);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  if (Serial.available()>0) {
    byte horn = Serial.read();
    readHorn = horn;
    if (readHorn == 1) {
      digitalWrite(buzzPin, HIGH);
      Serial.println("Horn On");
    }
    else if (readHorn == 0) {
      digitalWrite(buzzPin, LOW);
      Serial.println("Horn Off");
    }
  }

  unsigned long now = millis();
  
  //Lock
  uint8_t rfidtx = rfid.PCD_ReadRegister(rfid.TxControlReg);
  if (locked) {
    if ((rfidtx & 0x03) != 0x03) {
      locked = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Unlocked");
      //digitalWrite(buzzPin,HIGH); delay(1000); digitalWrite(buzzPin,LOW);
      delay(1000);
    }
    return;
  }
  bool irNow = (digitalRead(irPin) == HIGH);
  if (irNow && !prevIr) {
    unsigned long t = now;
    for (int i = 0; i < irCountThresh - 1; i++) {
      irTimes[i] = irTimes[i + 1];
    }
    irTimes[irCountThresh - 1] = t;
    int count = 0;
    for (int i = 0; i < irCountThresh; i++) {
      if (t - irTimes[i] <= irWin) count++;
    }
    if (count >= irCountThresh && lockEnabled) {
      locked = true;
      lockEnabled = false;
      sensorServo.write(90);
      flagServo.write(90);
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("** LOCKED **");
      //digitalWrite(buzzPin,HIGH); delay(200); digitalWrite(buzzPin,LOW); delay(200); digitalWrite(buzzPin,HIGH); delay(200); digitalWrite(buzzPin,LOW);
      delay(200);
    }
  }
  prevIr = irNow;

  //check accel & decel LED and Buzz
  if (now - lastAccelTime >= accelInterval) {
    lastAccelTime = now;
    accels();
    float accMag = sqrt(filtAx*filtAx + filtAy*filtAy);
    bool decel = (lastAccMag - accMag > decelThresh);
    if (decel) {
      digitalWrite(ledPin,  HIGH);
    }
    else {
      digitalWrite(ledPin, LOW);
      }
    lastAccMag = accMag;
  }

  //check dist & display dist & accel on LCD
  if (now - lastDistTime >= distInterval) {
    lastDistTime = now;
    measureDist();
    dispLCD();
  }
  
  //if forward dist<20, check right and left and flip flag to more open side for 2.5 sec
  if (flagHeld){
    if (millis() - flagHoldStart >= flagHoldDur){
      flagServo.write(90);
      flagHeld = false;
    }
  }
  else if (dist < 20 || scanPhase != 0) {
    doScan(millis());
  }
  else if (!isCentered){
    sensorServo.write(90);
    flagServo.write(90);
    isCentered = true;
  }
  
  //Unlock
  if (!irNow && !lockEnabled) {
    lockEnabled = true;
    digitalWrite(buzzPin, LOW);
    digitalWrite(ledPin, LOW);
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------

void measureDist() {
  digitalWrite(ustrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ustrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ustrig, LOW);
  duration = pulseIn(ussensor, HIGH);
  dist = (duration * .0343) / 2;
  //Serial.print("Distance: "); Serial.print(dist); Serial.println(" cm");
}

void dispLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print(dist);
  lcd.print(" cm      ");
  
  accXY = sqrt(filtAx*filtAx*filtAy*filtAy);
  lcd.setCursor(0,1);
  lcd.print("Accel:");
  lcd.print(accXY);
  lcd.print(" m/s^2    ");
}

void accels() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  rawAx = Wire.read() << 8 | Wire.read();
  rawAy = Wire.read() << 8 | Wire.read();
  rawAz = Wire.read() << 8 | Wire.read();
  
  ax_ms2 = (rawAx / scale) * g;
  ay_ms2 = (rawAy / scale) * g;
  az_ms2 = (rawAz / scale) * g;
 
  dynAx = ax_ms2;
  dynAy = ay_ms2;
  dynAz = az_ms2 - g;

  filtAx = alpha * (filtAx + dynAx - lastDynAx);
  filtAy = alpha * (filtAy + dynAy - lastDynAy);
  filtAz = alpha * (filtAz + dynAz - lastDynAz);

  lastDynAx = dynAx;
  lastDynAy = dynAy;
  lastDynAz = dynAz;
  //Serial.print("Dyn Accel [m/s²]  X: "); Serial.print(filtAx, 2); Serial.print("  Y: "); Serial.print(filtAy, 2); Serial.print("  Z: "); Serial.println(filtAz, 2);
}

void doScan(unsigned long now) {
  if (scanPhase == 0) {
    scanPhase = 1;
    phaseStart = now;
    lastTime = now;
    sweepPos = 90;
    sweepDir = +1;
  }

  else if (scanPhase == 1) {
  if (now - lastTime >= sweepInterval) {
      lastTime = now;
      sweepPos += sweepStep;
      if (sweepPos >= 180) {
        sweepPos = 180;
        scanPhase = 2;
        phaseStart = now;
      }
      sensorServo.write(sweepPos);
    }
  }

  else if (scanPhase == 2) {
    if (now - phaseStart >= pauseDuration) {
      scanPhase = 3;
      lastTime = now;
      sweepDir = -1;
      measureDist();
      ::distR = dist;
    }
  }

  else if (scanPhase == 3) {
    if (now - lastTime >= sweepInterval) {
      lastTime = now;
      sweepPos += sweepStep * sweepDir;
      if (sweepPos <= 0) {
        sweepPos = 0;
        scanPhase = 4;
        phaseStart = now;
      }
      sensorServo.write(sweepPos);
    }
  }

  else if (scanPhase == 4) {
    if (now - phaseStart >= pauseDuration) {
      scanPhase = 5;
      lastTime = now;
      sweepDir = +1;
      measureDist();
      ::distL = dist;
    }
  }

  else if (scanPhase == 5) {
    if (now - lastTime >= sweepInterval) {
      lastTime   = now;
      sweepPos  += sweepStep * sweepDir;
      if (sweepPos >= 90) {
        sweepPos   = 90;
        sensorServo.write(90);
        scanPhase  = 0;
        if (distR > distL){ 
          flagServo.write(180);
        }
        else{ 
          flagServo.write(0);
        }
        flagHeld      = true;
        flagHoldStart = millis();
      }
      else {
      sensorServo.write(sweepPos);
      }
    }
  }
}

