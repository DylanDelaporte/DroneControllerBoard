#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>
//#include <MPU6050.h>
#include <HMC5883L.h>
#include <NewPing.h>
#include <dht11.h>
#include <Servo.h>
#include "KalmanFilter.h"

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

int pinTemperatureSensor = 2;

int pinLED = 13;
int pinBuzzer = 1;

int vSonars[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float vVSpeed = 0;
float vHSpeed = 0;
int vDegrees = 0;
int vPressure = 0;

int cMotor1 = 0;
int cMotor2 = 0;
int cMotor3 = 0;
int cMotor4 = 0;

int cXAxis = 0;
int cYAxis = 0;

int cDegrees = 0;

int controlMode = 0;

int leftRightCalibrate = 0;
int frontBackCalibrate = 0;

float thrustMotors[4] = {0, 0, 0, 0};

float tPitchAccel = 0;
float tRollAccel = 0;

float pitchAccel = 0;
float rollAccel = 0;

int maxDistanceApproach = 20 + 15;

int lastRNumber = 0;
int cRNumber = 0;

int MIN_THRUST = 800;
int MAX_THRUST = 2000;

float CALIBRATE_ACCEL_PITCH = 0;
float CALIBRATE_ACCEL_ROLL = 0;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float kalPitch[2] = {0, 0};
float kalRoll[2] = {0, 0};

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

bool startFlashingLED = false;
bool isBuzzing = true;

bool isStabilizing = false;
bool isSleeping = false;

bool useSonars = false;

ThreadController controller = ThreadController();
Thread* outputInformations = new Thread();
Thread* lowSensors = new Thread();
Thread* detectConnection = new Thread();
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
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  Serial.println("Initializing : start");

  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);

  //buzzerSound(0);

  DHT11.attach(2);
  DHT11.read();

  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(9);

  mpu.initialize();

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  */

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
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

  /*
  Serial.println("Initializing : sensors");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G) && !compass.begin())
  {
    Serial.println("Could not find a valid MPU6050 sensor or compass.");
    delay(500);
  }
  */

  /*
  mpu.setAccelOffsetX(-425);
  mpu.setAccelOffsetY(-227);
  mpu.setAccelOffsetZ(1040);

  mpu.setThreshold(20);
  */

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(124, -118);

  outputInformations->setInterval(250);
  outputInformations->onRun(sendInformations);
  controller.add(outputInformations);

  lowSensors->setInterval(60000);
  lowSensors->onRun(analyzeLowSensors);
  controller.add(lowSensors);

  //detectConnection->setInterval(2000);
  //detectConnection->onRun(lostConnection);
  //controller.add(detectConnection);

  flashingLED->setInterval(1000);
  flashingLED->onRun(flashLED);

  Serial.println((controlMode == 1) ? "AUTOMATIC" : "MANUAL");
  Serial.println("Initializing : finish");
}

void loop() {
  checkCommand();
  defineDegrees();

  thrustMotors[0] = 0;
  thrustMotors[1] = 0;
  thrustMotors[2] = 0;
  thrustMotors[3] = 0;
  
  if (cMotor1 > 0) {
    //setDegrees();

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
  }

  if (useSonars) {
    analyzeSonars();

    cXAxis = ((vSonars[0][0] < maxDistanceApproach && cXAxis < 0) || (vSonars[1][0] < maxDistanceApproach && cXAxis > 0)) ? 0 : cXAxis;
    cYAxis = ((vSonars[2][0] < maxDistanceApproach && cYAxis > 0) || (vSonars[3][0] < maxDistanceApproach && cYAxis < 0)) ? 0 : cYAxis;

    if (vSonars[0][0] < maxDistanceApproach || vSonars[1][0] < maxDistanceApproach || vSonars[2][0] < maxDistanceApproach || vSonars[3][0] < maxDistanceApproach || vSonars[4][0] < maxDistanceApproach || vSonars[5][0] < maxDistanceApproach) {
      isSleeping = true;
    }
  }
  
  if (isStabilizing) {
    stabilizeDrone();
  }
  else if (isSleeping) {
    sleepDrone();
  }

  if (cMotor1 > 0) {
    thrustMotors[0] += cMotor1;
    thrustMotors[1] += cMotor2;
    thrustMotors[2] += cMotor3;
    thrustMotors[3] += cMotor4;
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

  if (flashingLED->shouldRun() && startFlashingLED)
    flashingLED->run();
}

void checkCommand()
{
  if (Serial.available())
  {
    int byteR = Serial.read();

    if (byteR == 13) {
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
  char part1 = command.charAt(0);
  String part2 = command.substring(2);

  if (part1 == 'P') {
    //Serial.println("POWER");

    int comma1 = part2.indexOf("|");
    int comma2 = part2.indexOf("|", comma1 + 1);
    int comma3 = part2.indexOf("|", comma2 + 1);
    int comma4 = part2.indexOf("|", comma3 + 1);

    cMotor1 = String(part2.substring(0, comma1)).toInt();
    cMotor2 = cMotor1;
    cMotor3 = cMotor1;
    cMotor4 = cMotor1;

    cDegrees = String(part2.substring(comma1 + 1, comma2)).toInt();

    cXAxis = String(part2.substring(comma2 + 1, comma3)).toInt();
    cYAxis = String(part2.substring(comma3 + 1, comma4)).toInt();

    lostConnection(String(part2.substring(comma4 + 1)).toInt());

    /*
    Serial.println(cMotor1);
    Serial.println(cMotor2);
    Serial.println(cMotor3);
    Serial.println(cMotor4);

    Serial.println(cDegrees);

    Serial.println(cXAxis);
    Serial.println(cYAxis);
    */
  }
  else if (part1 == 'M') {
    int comma = part2.indexOf("|");

    if (part2.substring(0, comma).toInt() == 1) {
      controlMode = 1;
    }
    else {
      controlMode = 0;
    }

    lostConnection(String(part2.substring(comma + 1)).toInt());
  }
  else if (part1 == 'C') {
    int comma1 = part2.indexOf("|");
    //int comma2 = part2.indexOf("|", comma1 + 1);

    leftRightCalibrate = String(part2.substring(0, comma1)).toInt();
    frontBackCalibrate = String(part2.substring(comma1 + 1)).toInt();

    //lostConnection(String(part2.substring(comma2 + 1)).toInt());

    //Serial.println("D C Y");
  }
  else if (part1 == 'A') {
    int comma = part2.indexOf("|");

    if (part2.substring(0, comma).equalsIgnoreCase("Y")) {
      CALIBRATE_ACCEL_PITCH = -tPitchAccel;
      CALIBRATE_ACCEL_ROLL = -tRollAccel;
    }
  }
  else if (part1 == 'S') {
    int comma = part2.indexOf("|");

    if (part2.substring(0, comma).equalsIgnoreCase("Y")) {
      useSonars = true;
    }
    else
    {
      useSonars = false;
    }

    lostConnection(String(part2.substring(comma + 1)).toInt());
  }
  else if (part1 == 'L') {
    int comma = part2.indexOf("|");

    if (part2.substring(0, comma).equalsIgnoreCase("Y")) {
      startFlashingLED = true;
    }
    else
    {
      startFlashingLED = false;
    }

    lostConnection(String(part2.substring(comma + 1)).toInt());
  }
  else
  {
    Serial.println("D NF Y");
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

  vDegrees = heading * 180 / M_PI;
  
  //Serial.println(vDegrees);
  
  if (firstTime) {
    cDegrees = vDegrees;
    firstTime = false;
  }
}

void setDegrees() {
  if (cDegrees > vDegrees) {
    thrustMotors[0] += 2;
    thrustMotors[1] -= 2;
    thrustMotors[2] -= 2;
    thrustMotors[3] += 2;
  }
  else if (cDegrees < vDegrees)
  {
    thrustMotors[0] -= 2;
    thrustMotors[1] += 2;
    thrustMotors[2] += 2;
    thrustMotors[3] -= 2;
  }
}

void manualAxis() {
  if (cXAxis > 0) {
    thrustMotors[0] += (((float)cXAxis / (float)45) * 10);
    thrustMotors[2] += (((float)cXAxis / (float)45) * 10);
  }
  else if (cXAxis < 0) {
    thrustMotors[1] += (((float) - cXAxis / (float)45) * 10);
    thrustMotors[3] += (((float) - cXAxis / (float)45) * 10);
  }

  if (cYAxis > 0) {
    thrustMotors[0] += (((float)cYAxis / (float)45) * 10);
    thrustMotors[1] += (((float)cYAxis / (float)45) * 10);
  }
  else if (cYAxis < 0) {
    thrustMotors[2] += (((float) - cYAxis / (float)45) * 10);
    thrustMotors[3] += (((float) - cYAxis / (float)45) * 10);
  }
}

void analyzeSensors() {
  /*
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  pitchAccel = (-(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI) + CALIBRATE_ACCEL_PITCH;
  rollAccel = ((atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI) + CALIBRATE_ACCEL_ROLL;

  float nKalPitch = (float)((kalmanY.update(pitchAccel, normGyro.YAxis) + kalPitch[1]) / 2);
  float nKalRoll = (float)((kalmanX.update(rollAccel, normGyro.XAxis) + kalRoll[1]) / 2);

  kalPitch[0] = (float)((int)(nKalPitch * 10)) / 10;
  kalRoll[0] = (float)((int)(nKalRoll * 10)) / 10;;

  kalPitch[1] = kalPitch[0];
  kalRoll[1] = kalRoll[0];

  Serial.print(kalPitch[0]);
  Serial.print(" - ");
  Serial.println(kalRoll[0]);
  */

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

    tPitchAccel = (ypr[1] * 180 / M_PI);
    tRollAccel = (ypr[2] * 180 / M_PI);

    pitchAccel = tPitchAccel + CALIBRATE_ACCEL_PITCH;
    rollAccel = tRollAccel + CALIBRATE_ACCEL_ROLL;

    //Serial.print(rollAccel);
    //Serial.print("\t");
    //Serial.println(pitchAccel);
  }

  //vVSpeed = (normAccel.ZAxis - 9.81) * 0.01;
  //vHSpeed = sqrt(pow(normAccel.XAxis, 2) + pow(normAccel.YAxis, 2)) * 0.01;

  //Serial.print(vVSpeed);
  //Serial.print("-");
  //Serial.println(vHSpeed);

  //Serial.print(" Yg = ");
  //Serial.print(normGyro.YAxis);
  //Serial.print(" Zg = ");
  //Serial.print(normGyro.ZAxis);

  if (useSonars) {
    cXAxis = ((vSonars[0][0] < maxDistanceApproach && cXAxis < 0) || (vSonars[1][0] < maxDistanceApproach && cXAxis > 0)) ? 0 : cXAxis;
    cYAxis = ((vSonars[2][0] < maxDistanceApproach && cYAxis > 0) || (vSonars[3][0] < maxDistanceApproach && cYAxis < 0)) ? 0 : cYAxis;

    if (vSonars[0][0] < maxDistanceApproach || vSonars[1][0] < maxDistanceApproach || vSonars[2][0] < maxDistanceApproach || vSonars[3][0] < maxDistanceApproach || vSonars[4][0] < maxDistanceApproach || vSonars[5][0] < maxDistanceApproach) {
      isSleeping = true;
    }
  }

  if (vHSpeed < -0.04) {
    isStabilizing = true;
  }

  /*
  Serial.print(vDegrees);
  Serial.print("-");
  Serial.print(pitchAccel);
  Serial.print("-");
  Serial.print(rollAccel);
  Serial.print("-");
  Serial.print(thrustMotors[0] + cMotor1);
  Serial.print("-");
  Serial.print(map((thrustMotors[0] + cMotor1), 0, 100, MIN_THRUST, MAX_THRUST));
  Serial.print("-");
  Serial.print(map((thrustMotors[1] + cMotor2), 0, 100, MIN_THRUST, MAX_THRUST));
  Serial.print("-");
  Serial.print(map((thrustMotors[2] + cMotor3), 0, 100, MIN_THRUST, MAX_THRUST));
  Serial.print("-");
  Serial.println(map((thrustMotors[3] + cMotor4), 0, 100, MIN_THRUST, MAX_THRUST));
  */
}

void automaticAxis() {
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

    tPitchAccel = (ypr[1] * 180 / M_PI);
    tRollAccel = (ypr[2] * 180 / M_PI);

    pitchAccel = tPitchAccel + CALIBRATE_ACCEL_PITCH;
    rollAccel = tRollAccel + CALIBRATE_ACCEL_ROLL;

    //Serial.print(rollAccel);
    //Serial.print("\t");
    //Serial.println(pitchAccel);
  }

  if (cXAxis < 0 || cXAxis > 0) {
    if (cXAxis > rollAccel) {
      thrustMotors[0] += 5;
      thrustMotors[2] += 5;
    }
    else if (cXAxis < rollAccel) {
      thrustMotors[1] += -5;
      thrustMotors[3] += -5;
    }
  }
  else
  {
    if (rollAccel > 0) {
      thrustMotors[0] += rollAccel;
      thrustMotors[2] += rollAccel;
    }
    else if (rollAccel < 0) {
      thrustMotors[1] += -rollAccel;
      thrustMotors[3] += -rollAccel;
    }
  }

  if (cYAxis < 0 || cYAxis > 0) {
    if (cYAxis > pitchAccel) {
      thrustMotors[0] += 5;
      thrustMotors[1] += 5;
    }
    else if (cYAxis < pitchAccel) {
      thrustMotors[2] += -5;
      thrustMotors[3] += -5;
    }
  }
  else
  {
    if (pitchAccel > 0) {
      thrustMotors[0] += pitchAccel;
      thrustMotors[1] += pitchAccel;
    }
    else if (pitchAccel < 0) {
      thrustMotors[2] += -pitchAccel;
      thrustMotors[3] += -pitchAccel;
    }
  }
}

void stabilizeDrone() {
  cXAxis = 0;
  cYAxis = 0;

  if (vHSpeed > 0) {
    cMotor1 -= 1;
    cMotor2 -= 1;
    cMotor3 -= 1;
    cMotor4 -= 1;
  }
  else if (vHSpeed < 0)
  {
    cMotor1 += 1;
    cMotor2 += 1;
    cMotor3 += 1;
    cMotor4 += 1;
  }
  else
  {
    isStabilizing = false;
  }
}

void sleepDrone() {
  cXAxis = 0;
  cYAxis = 0;

  if (vSonars[5][0] < 20) {
    cMotor1 += 5;
    cMotor2 += 5;
    cMotor3 += 5;
    cMotor4 += 5;
  }
  else if (vSonars[5][0] == 0 && vSonars[5][1] < 20) {
    cMotor1 = 0;
    cMotor2 = 0;
    cMotor3 = 0;
    cMotor4 = 0;

    isSleeping = false;
  }
  else
  {
    if (vHSpeed > 0.01) {
      cMotor1 -= 1;
      cMotor2 -= 1;
      cMotor3 -= 1;
      cMotor4 -= 1;
    }
    else if (vHSpeed < 0.01) {
      cMotor1 += 1;
      cMotor2 += 1;
      cMotor3 += 1;
      cMotor4 += 1;
    }
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
    digitalWrite(pinLED, HIGH);

    if (cMotor1 <= 20) {
      //cMotor1 = 0;
    }
  }
  else {
    digitalWrite(pinLED, LOW);
  }
}

void sendInformations() {
  Serial.print("D D ");
  Serial.print(vSonars[0][0]);
  Serial.print("|");
  Serial.print(vSonars[1][0]);
  Serial.print("|");
  Serial.print(vSonars[2][0]);
  Serial.print("|");
  Serial.print(vSonars[3][0]);
  Serial.print("|");
  Serial.print(vSonars[4][0]);
  Serial.print("|");
  Serial.print(vSonars[5][0]);
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

void flashLED() {
  Serial.println("flashLED");

  digitalWrite(pinLED, HIGH);

  delay(50);

  digitalWrite(pinLED, LOW);

  delay(100);

  digitalWrite(pinLED, HIGH);

  delay(50);

  digitalWrite(pinLED, LOW);
}

void buzzerSound(int type) {
  if (!isBuzzing) {
    if (type == 0) {
      digitalWrite(pinBuzzer, HIGH);
      delay(100);
      digitalWrite(pinBuzzer, LOW);
      delay(400);
      digitalWrite(pinBuzzer, HIGH);
      delay(50);
      digitalWrite(pinBuzzer, LOW);
      delay(100);
      digitalWrite(pinBuzzer, HIGH);
      delay(50);
      digitalWrite(pinBuzzer, LOW);
      delay(100);
      digitalWrite(pinBuzzer, HIGH);
      delay(200);
      digitalWrite(pinBuzzer, LOW);
      delay(200);
    }
    else
    {
      delay(type * 10);
      digitalWrite(pinBuzzer, HIGH);
      delay(200);
      digitalWrite(pinBuzzer, LOW);
    }

    isBuzzing = false;
  }
}




