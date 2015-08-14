#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <Thread.h>
#include <ThreadController.h>
#include <HMC5883L.h>
#include <NewPing.h>
#include <dht11.h>
#include <Servo.h>

String command = "";

int pinMotor1 = 3;
int pinMotor2 = 5;
int pinMotor3 = 6;
int pinMotor4 = 9;

int pinTriggerSonar = 4;
int pinLeftSonar = 7;
int pinRightSonar = 8;
int pinFrontSonar = 10;
int pinBackSonar = 11;
int pinUpSonar = 12;
int pinDownSonar = 13;

int pinTemperatureSensor = 4;

int pinLED = 13;

int vSonars[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float vVSpeed = 0;
float vHSpeed = 0;
int vDegrees = 0;
int vPressure = 0;

int cMotor = 0;

int cXAxis = 0;
int cYAxis = 0;

int cDegrees = 0;

int controlMode = 0;

int leftRightCalibrate = 0;
int frontBackCalibrate = 0;

float thrustMotors[4] = {0, 0, 0, 0};

float pitchAccel[2] = {0, 0};
float rollAccel[2] = {0, 0};

int maxDistanceApproach = 20 + 15;

int lastRNumber = 0;
int cRNumber = 0;

int cCommand = 0;
int lastCCommand = 0;

bool lostConnectionCommand = false;
bool lostConnectionWifi = false;

bool informationCommand = false;

int countSendCommand = 0;

int MIN_THRUST = 800;
int MAX_THRUST = 2000;

float CALIBRATE_ACCEL_PITCH = 0;
float CALIBRATE_ACCEL_ROLL = 0;

int axisSensibility = 10;
int rotationSensibility = 2;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;

float ypr[3];

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

bool firstTime = true;

bool isFlashingLED = false;
bool isBuzzing = true;

bool isStabilizing = false;

bool isSleeping = false;
bool isSleepingDemand = false;

bool useSonars = false;
bool useSensors = true;

int countTest = 0;

ThreadController controller = ThreadController();
Thread* outputInformations = new Thread();
Thread* lowSensors = new Thread();
Thread* lostConnectionTest = new Thread();
Thread* sleepingProcess = new Thread();
Thread* flashingLED = new Thread();

MPU6050 mpu;
HMC5883L compass;

NewPing sonars[6] = {
  NewPing(pinTriggerSonar, pinLeftSonar),
  NewPing(pinTriggerSonar, pinRightSonar),
  NewPing(pinTriggerSonar, pinFrontSonar),
  NewPing(pinTriggerSonar, pinBackSonar),
  NewPing(pinTriggerSonar, pinUpSonar),
  NewPing(pinTriggerSonar, pinDownSonar)
};

dht11 DHT11;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  if (useSensors) {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
  }

  Serial.begin(115200);

  //Serial.println("Initializing : start");

  pinMode(pinLED, OUTPUT);

  //DHT11.attach(pinTemperatureSensor);
  //DHT11.read();

  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(9);

  if (useSensors) {
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      //Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      //Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      //Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      //Serial.println(F(")"));
    }

    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_30HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
    compass.setOffset(124, -118);
  }

  outputInformations->setInterval(250);
  outputInformations->onRun(sendInformations);
  controller.add(outputInformations);

  lowSensors->setInterval(60000);
  lowSensors->onRun(analyzeLowSensors);
  controller.add(lowSensors);

  lostConnectionTest->setInterval(1000);
  lostConnectionTest->onRun(lostConnectionTime);
  controller.add(lostConnectionTest);

  sleepingProcess->setInterval(200);
  sleepingProcess->onRun(sleepDrone);

  flashingLED->setInterval(1000);
  flashingLED->onRun(flashLED);
  
  //Serial.println("Initializing : finish");
}

void loop() {
  checkCommand();

  if (useSonars) {
    analyzeSonars();

    cXAxis = ((vSonars[0][0] < maxDistanceApproach && cXAxis < 0) || (vSonars[1][0] < maxDistanceApproach && cXAxis > 0)) ? 0 : cXAxis;
    cYAxis = ((vSonars[2][0] < maxDistanceApproach && cYAxis > 0) || (vSonars[3][0] < maxDistanceApproach && cYAxis < 0)) ? 0 : cYAxis;
    
    if (vSonars[0][0] < maxDistanceApproach || vSonars[1][0] < maxDistanceApproach || vSonars[2][0] < maxDistanceApproach || vSonars[3][0] < maxDistanceApproach || vSonars[4][0] < maxDistanceApproach || vSonars[5][0] < maxDistanceApproach) {
      isSleeping = true;
    }
  }

  if (vHSpeed < -0.04) {
    isStabilizing = true;
  }

  if (sleepingProcess->shouldRun() && (isSleeping || isSleepingDemand))
    sleepingProcess->run();

  if (useSensors) {
    defineDegrees();
    definePitchRoll();
  }

  thrustMotors[0] = 0;
  thrustMotors[1] = 0;
  thrustMotors[2] = 0;
  thrustMotors[3] = 0;

  if (cMotor > 0) {
    setDegrees();

    if (controlMode == 1) {
      automaticAxis();
    }
    else
    {
      manualAxis();
    }

    if (leftRightCalibrate > 0) {
      thrustMotors[1] += leftRightCalibrate;
      thrustMotors[3] += leftRightCalibrate;
    }
    else if (leftRightCalibrate < 0) {
      thrustMotors[0] += -leftRightCalibrate;
      thrustMotors[2] += -leftRightCalibrate;
    }

    if (frontBackCalibrate > 0) {
      thrustMotors[0] += frontBackCalibrate;
      thrustMotors[1] += frontBackCalibrate;
    }
    else if (frontBackCalibrate < 0) {
      thrustMotors[2] += -frontBackCalibrate;
      thrustMotors[3] += -frontBackCalibrate;
    }

    thrustMotors[0] += cMotor;
    thrustMotors[1] += cMotor;
    thrustMotors[2] += cMotor;
    thrustMotors[3] += cMotor;

    if (thrustMotors[0] < 0 || thrustMotors[1] < 0 || thrustMotors[2] < 0 || thrustMotors[3] < 0) {
      thrustMotors[0] = thrustMotors[1] = thrustMotors[2] = thrustMotors[3] = 0;
    }
  }

  /*
  Serial.print(thrustMotors[0]);
  Serial.print(" - ");
  Serial.print(thrustMotors[1]);
  Serial.print(" - ");
  Serial.print(thrustMotors[2]);
  Serial.print(" - ");
  Serial.println(thrustMotors[3]);
  */

  motor1.writeMicroseconds(map(thrustMotors[0], 0, 100, MIN_THRUST, MAX_THRUST));
  motor2.writeMicroseconds(map(thrustMotors[1], 0, 100, MIN_THRUST, MAX_THRUST));
  motor3.writeMicroseconds(map(thrustMotors[2], 0, 100, MIN_THRUST, MAX_THRUST));
  motor4.writeMicroseconds(map(thrustMotors[3], 0, 100, MIN_THRUST, MAX_THRUST));

  controller.run();

  if (flashingLED->shouldRun() && isFlashingLED)
    flashingLED->run();
}

void checkCommand()
{
  if (Serial.available())
  {
    int byteR = Serial.read();

    if (byteR == 10) {
      parseCommand(command);
      command = "";
    }
    else
    {
      command += char(byteR);
    }
  }
}

void parseCommand(String command) {
  cCommand++;

  char part1 = command.charAt(0);

  int space1 = command.indexOf(" ");
  int space2 = command.indexOf(" ", space1 + 1);

  String part2 = command.substring(space1 + 1, space2);

  lostConnection(String(command.substring(space2 + 1)).toInt());

  if (part1 == 'P') {
    int comma1 = part2.indexOf("|");
    int comma2 = part2.indexOf("|", comma1 + 1);
    int comma3 = part2.indexOf("|", comma2 + 1);

    if (!isSleeping) {
      cMotor = String(part2.substring(0, comma1)).toInt();

      cDegrees = String(part2.substring(comma1 + 1, comma2)).toInt();

      cXAxis = String(part2.substring(comma2 + 1, comma3)).toInt();
      cYAxis = String(part2.substring(comma3 + 1)).toInt();
    }

    /*
    Serial.println(cMotor);

    Serial.println(cDegrees);

    Serial.println(cXAxis);
    Serial.println(cYAxis);
    */
  }
  else if (part1 == 'M') {
    if (part2.toInt() == 1) {
      controlMode = 1;
    }
    else {
      controlMode = 0;
    }
  }
  else if (part1 == 'C') {
    int comma1 = part2.indexOf("|");

    leftRightCalibrate = String(part2.substring(0, comma1)).toInt();
    frontBackCalibrate = String(part2.substring(comma1 + 1)).toInt();

    //Serial.println("D C Y");
  }
  else if (part1 == 'A') {
    if (part2.equalsIgnoreCase("Y")) {
      CALIBRATE_ACCEL_PITCH = -pitchAccel[0];
      CALIBRATE_ACCEL_ROLL = -rollAccel[0];
    }
  }
  else if (part1 == 'O') {
    int comma1 = part2.indexOf("|");

    int tempAxisSensibility = String(part2.substring(0, comma1)).toInt();
    int tempRotationSensibility = String(part2.substring(comma1 + 1)).toInt();

    if (tempAxisSensibility >= 5) {
      axisSensibility = tempAxisSensibility;
    }

    if (tempRotationSensibility >= 0) {
      rotationSensibility = tempRotationSensibility;
    }
  }
  else if (part1 == 'I') {
    if (part2.equalsIgnoreCase("Y")) {
      informationCommand = true;
      countSendCommand = 0;
    }
  }
  else if (part1 == 'H') {
    if (part2.equalsIgnoreCase("Y")) {
      isSleepingDemand = true;
    }
    else {
      isSleepingDemand = false;
    }
  }
  else if (part1 == 'S') {
    if (part2.equalsIgnoreCase("Y")) {
      useSonars = true;
    }
    else
    {
      useSonars = false;
    }
  }
  else if (part1 == 'L') {
    if (part2.equalsIgnoreCase("Y")) {
      isFlashingLED = true;
    }
    else
    {
      isFlashingLED = false;
    }
  }
  else
  {
    //Serial.println("D NF Y");
  }
}

void defineDegrees() {
  Vector normCompass = compass.readNormalize();

  float heading = atan2(normCompass.YAxis, normCompass.XAxis);
  float declinationAngle = (1.0 + (18.0 / 60.0)) / (180 / M_PI);

  heading += declinationAngle;

  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  
  if (firstTime) {
    vDegrees = heading * 180 / M_PI;
    
    cDegrees = vDegrees;
    firstTime = false;
  }
  else {
    if (pitchAccel[1] < 1 && pitchAccel[1] > -1 && rollAccel[1] < 1 && rollAccel[1] > -1) {
      vDegrees = heading * 180 / M_PI;
    }
  }
}

void setDegrees() {
  if (pitchAccel[1] < 1 && pitchAccel[1] > -1 && rollAccel[1] < 1 && rollAccel[1] > -1) {
    if (cDegrees > vDegrees) {
      if ((cDegrees - vDegrees) < ((360 - cDegrees) + vDegrees)) {
        thrustMotors[0] += rotationSensibility;
        thrustMotors[1] -= rotationSensibility;
        thrustMotors[2] -= rotationSensibility;
        thrustMotors[3] += rotationSensibility;
      }
      else {
        thrustMotors[0] -= rotationSensibility;
        thrustMotors[1] += rotationSensibility;
        thrustMotors[2] += rotationSensibility;
        thrustMotors[3] -= rotationSensibility;
      }
    }
    else if (cDegrees < vDegrees)
    {
      if ((vDegrees - cDegrees) > ((360 - vDegrees) + cDegrees)) {
        thrustMotors[0] += rotationSensibility;
        thrustMotors[1] -= rotationSensibility;
        thrustMotors[2] -= rotationSensibility;
        thrustMotors[3] += rotationSensibility;
      }
      else {
        thrustMotors[0] -= rotationSensibility;
        thrustMotors[1] += rotationSensibility;
        thrustMotors[2] += rotationSensibility;
        thrustMotors[3] -= rotationSensibility;
      }
    }
  }
}

void definePitchRoll() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitchAccel[0] = (ypr[1] * 180 / M_PI);
    rollAccel[0] = (ypr[2] * 180 / M_PI);

    pitchAccel[1] = pitchAccel[0] + CALIBRATE_ACCEL_PITCH;
    rollAccel[1] = rollAccel[0] + CALIBRATE_ACCEL_ROLL;

    //Serial.print(rollAccel);
    //Serial.print("\t");
    //Serial.println(pitchAccel);
  }
}

void automaticAxis() {
  if (cXAxis < 0 || cXAxis > 0) {
    if (cXAxis > rollAccel[1]) {
      thrustMotors[0] += 5;
      thrustMotors[2] += 5;
    }
    else if (cXAxis < rollAccel[1]) {
      thrustMotors[1] += -5;
      thrustMotors[3] += -5;
    }
  }
  else
  {
    if (rollAccel[1] > 0) {
      thrustMotors[2] += rollAccel[1];
      thrustMotors[3] += rollAccel[1];
    }
    else if (rollAccel[1] < 0) {
      thrustMotors[0] += -rollAccel[1];
      thrustMotors[1] += -rollAccel[1];
    }
  }

  if (cYAxis < 0 || cYAxis > 0) {
    if (cYAxis > pitchAccel[1]) {
      thrustMotors[0] += 5;
      thrustMotors[1] += 5;
    }
    else if (cYAxis < pitchAccel[1]) {
      thrustMotors[2] += -5;
      thrustMotors[3] += -5;
    }
  }
  else
  {
    if (pitchAccel[1] > 0) {
      thrustMotors[0] += pitchAccel[1];
      thrustMotors[2] += pitchAccel[1];
    }
    else if (pitchAccel[1] < 0) {
      thrustMotors[1] += -pitchAccel[1];
      thrustMotors[3] += -pitchAccel[1];
    }
  }
}

void manualAxis() {
  if (cXAxis > 0) {
    thrustMotors[0] += (((float)cXAxis / (float)45) * axisSensibility);
    thrustMotors[2] += (((float)cXAxis / (float)45) * axisSensibility);
  }
  else if (cXAxis < 0) {
    thrustMotors[1] += (((float) - cXAxis / (float)45) * axisSensibility);
    thrustMotors[3] += (((float) - cXAxis / (float)45) * axisSensibility);
  }

  if (cYAxis > 0) {
    thrustMotors[0] += (((float)cYAxis / (float)45) * axisSensibility);
    thrustMotors[1] += (((float)cYAxis / (float)45) * axisSensibility);
  }
  else if (cYAxis < 0) {
    thrustMotors[2] += (((float) - cYAxis / (float)45) * axisSensibility);
    thrustMotors[3] += (((float) - cYAxis / (float)45) * axisSensibility);
  }
}

void stabilizeDrone() {
  cXAxis = 0;
  cYAxis = 0;

  if (vHSpeed > 0) {
    cMotor -= 1;
  }
  else if (vHSpeed < 0)
  {
    cMotor += 1;
  }
  else
  {
    isStabilizing = false;
  }
}

void sleepDrone() {
  cXAxis = 0;
  cYAxis = 0;

  if (cMotor <= 20) {
    cMotor = 0;
  }
  else if (cMotor <= 50) {
    cMotor -= 0.5;
  }
  else {
    cMotor -= 0.2;
  }
}

void analyzeSonars() {
  int nValueSonar = 0;

  for (int i = 0; i < 6; i++) {
    nValueSonar = sonars[i].ping() / US_ROUNDTRIP_CM;

    if (nValueSonar == 0) {
      if (vSonars[i][0] < 7) {
        vSonars[i][1] = vSonars[i][0];
        vSonars[i][0] = 0;
      }
      else if (vSonars[i][0] > 350) {
        vSonars[i][1] = vSonars[i][0];
        vSonars[i][0] = 1234;
      }
    }
    else if (nValueSonar < (vSonars[i][0] - 15) || nValueSonar > (vSonars[i][0] + 15))
    {
      if (vSonars[i][2] == 5) {
        vSonars[i][1] = vSonars[i][0];
        vSonars[i][0] = nValueSonar;
        vSonars[i][2] = 0;
      }
      else
      {
        vSonars[i][2]++;
      }
    }
    else
    {
      vSonars[i][1] = vSonars[i][0];
      vSonars[i][0] = ((nValueSonar + vSonars[i][0]) / 2);
    }
  }
}

void analyzeLowSensors() {
  DHT11.read();
}

void lostConnection(int rNumber) {
  if (rNumber == lastRNumber) {
    cRNumber++;
  }
  else {
    cRNumber = 0;
    lastRNumber = rNumber;
  }

  if (cRNumber >= 4) {
    lostConnectionCommand = true;
  }
  else {
    lostConnectionCommand = false;
  }
}

void lostConnectionTime() {
  if (cCommand == lastCCommand) {
    lostConnectionWifi = true;
  }
  else {
    lostConnectionWifi = false;
  }

  lastCCommand = cCommand;

  if (lostConnectionWifi || lostConnectionCommand) {
    Serial.println("D L LOST");

    digitalWrite(pinLED, HIGH);

    isSleeping = true;
  }
  else {
    digitalWrite(pinLED, LOW);

    isSleeping = false;
  }
}

void sendInformations() {
  if (informationCommand) {
    if (countSendCommand > 2) {
      informationCommand = false;
    }
    else {
      Serial.print("D I ");
      Serial.print(controlMode);
      Serial.print("|");
      Serial.print(useSonars ? 1 : 0);
      Serial.print("|");
      Serial.print(axisSensibility);
      Serial.print("|");
      Serial.print(rotationSensibility);
      Serial.print("|");
      Serial.println(isFlashingLED ? 1 : 0);

      countSendCommand++;
    }
  }
  else {
    countTest++;

    Serial.print("D D ");
    //Serial.print(vSonars[0][0]);
    //Serial.print(countTest);
    Serial.print(thrustMotors[0]);
    Serial.print("|");
    //Serial.print(vSonars[1][0]);
    Serial.print(thrustMotors[1]);
    Serial.print("|");
    //Serial.print(vSonars[2][0]);
    Serial.print(thrustMotors[2]);
    Serial.print("|");
    //Serial.print(vSonars[3][0]);
    Serial.print(thrustMotors[3]);
    Serial.print("|");
    Serial.print(pitchAccel[1]);
    Serial.print("|");
    Serial.print(rollAccel[1]);
    Serial.print("|");
    Serial.print(vVSpeed);
    Serial.print("|");
    Serial.print(vHSpeed);
    Serial.print("|");
    Serial.print(vDegrees);
    Serial.print("|");
    Serial.print("12");
    Serial.print("|");
    Serial.print("0");
    Serial.print("|");
    Serial.print(DHT11.temperature);
    Serial.print("|");
    Serial.print(DHT11.humidity);
    Serial.println();
  }
}

void flashLED() {
  digitalWrite(pinLED, HIGH);

  delay(50);

  digitalWrite(pinLED, LOW);

  delay(100);

  digitalWrite(pinLED, HIGH);

  delay(50);

  digitalWrite(pinLED, LOW);
}




