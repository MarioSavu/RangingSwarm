#include <SPI.h>
#include "DW1000Ranging.h"

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

#define lDir 3 //pinul 0 este pinul de direcţie pentru roata stângă
#define lSpeed 5 //pinul 3 este pinul cu care putem controla viteza motorului stâng
#define rSpeed 6 //pinul 4 este pinul cu care putem controla viteza motorului drept 
#define rDir 4 //pinul 1 este pinul de direcţie pentru roata dreaptă

#define maxPWM 255 //valoarea maximă a PWM-ului
#define minPWM 50 //valoarea mminimă a PWM-ului
#define motorForward LOW //valoarea logică a pinului de direcţie ca motorul să se rotească înainte
#define motorBackward HIGH //valoarea logică a pinului de direcţie ca motorul să se rotească înapoi

#define START_DELAY 2 // in seconds

int precision = 2;
int incomingData = 0; //variabila de tip integer în care se memoriază caracterul recepţionat pe Bluetooth
byte incomingAngle = 0;
int motorSpeed = maxPWM; //variabila de tip integer în care se memoriază viteza actuală a motoarelor, starea iniţială este 255.
int debug = 0; //variabila de tip integer în care activeayă funcţiile de debug
int turningSpeed= 200; //variabila de tip integer în care se memoriază viteya motoarelor pentru rotiriile spre dreapta si stânga
int serialEventDone = 0; //variabila de tip integer în care se memoriază dacă s-a recepţionat cevpe serial
int maxSpeed = minPWM; // ???
int desiredAngle = 0;
int turnFlag = 0;
int printMe;

float globalDistanceVar = 0;
unsigned long lastUpdate = 0;

void setup()
{
  pinMode(lSpeed, OUTPUT); //definirea pinului lSpeed ca iesire
  pinMode(rSpeed, OUTPUT); //definirea pinului rSpeed ca iesire
  pinMode(lDir, OUTPUT); //definirea pinului lDir ca iesire
  analogWrite(lSpeed, 255); //setarea PWM-ului la valoarea de 255 pentru lSpeed
  pinMode(rDir, OUTPUT); //definirea pinului rDir ca iesire
  analogWrite(rSpeed, 255); //setarea PWM-ului la valoarea de 255 pentru rSpeed
  brake();
  Serial.begin(115200); //pornirea comunicării pe serial
  Serial.println("Serial Running.");
  if(debug)
  {
    Serial.println("Serial Running.");
  };
  // setupMPU6050();
  delay(1000 * START_DELAY);

  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  
  //we start the module as a tag
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  DW1000Ranging.loop();
  
  if(globalDistanceVar > 0.9 && (millis() - lastUpdate < 250)) {
    motorDrive(3);
  }
  else {
    brake();
  }
}

void newRange() {
  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
  globalDistanceVar = DW1000Ranging.getDistantDevice()->getRange();
  lastUpdate = millis();
}

void newDevice(DW1000Device* device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
  brake();
}

#if 0
void loop()
{
  motorDrive(1);
  delay(2000);
  motorDrive(2);
  delay(2000);
  motorDrive(0);  //stop the motors
  delay(2000);
  motorDrive(3);
  delay(600);
  brake();
  delay(1400);
  motorDrive(4);
  delay(600);
  brake();
  delay(1400);
}
#endif

void motorDrive(int mode)
{
  switch (mode)
  {
  case 0:  //stop
    brake();
    if(debug)
    {
      Serial.println(motorSpeed)  ;
    };
    break;
  case 1:  //forward
    digitalWrite(lDir, motorForward);
    digitalWrite(rDir, motorForward);
    if(accelerate() == 1)
    {
      // analogWrite(lSpeed, minPWM);
      // analogWrite(rSpeed, minPWM);
    }
    if(debug)
    {
      Serial.println(motorSpeed)  ;
    };
    break;

  case 2:  //backward
    digitalWrite(lDir, motorBackward);
    digitalWrite(rDir, motorBackward);

    if(accelerate() == 1)
    {
      // analogWrite(lSpeed, minPWM);
      // analogWrite(rSpeed, minPWM);
    }
    if(debug)
    {
      Serial.println(motorSpeed)  ;
    };
    break;
  case 3:  //left
    digitalWrite(lDir, motorBackward);
    digitalWrite(rDir, motorForward);
    motorSpeed = turningSpeed;
    analogWrite(lSpeed, motorSpeed);
    analogWrite(rSpeed, motorSpeed);
    //delay(50);
    //brake();
    break;
  case 4:  //right
    digitalWrite(lDir, motorForward);
    digitalWrite(rDir, motorBackward);
    motorSpeed = turningSpeed;
    analogWrite(lSpeed, motorSpeed);
    analogWrite(rSpeed, motorSpeed);
    //delay(50);
    //brake();
    break;
  default:
    break;
  }
  Serial.write(0); // acknowledge for client that command has been executed
}
int brake()
{
  // for(motorSpeed; motorSpeed<maxPWM; motorSpeed++) 
  // {
    analogWrite(lSpeed, maxPWM);
    analogWrite(rSpeed, maxPWM);
    // delayMicroseconds(500);
  // }
  digitalWrite(lSpeed, HIGH);
  digitalWrite(rSpeed, HIGH);

  if(debug)
  {
    Serial.println("brake 1")  ;
  };
  return 1;
}
int accelerate() 
{
  // for(motorSpeed = maxPWM; motorSpeed>minPWM; motorSpeed--)
  // {
    analogWrite(lSpeed, maxSpeed);
    analogWrite(rSpeed, maxSpeed);
    // delay(2);
  // }

  if(debug)
  {
    Serial.println("accelerate 1")  ;
  };
  return 1;
}
