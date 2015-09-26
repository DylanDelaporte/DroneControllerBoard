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

//PINS
int pinMotor1 = 9;
int pinMotor2 = 6;
int pinMotor3 = 3;
int pinMotor4 = 5;

int pinTriggerSonar = 4;
int pinLeftSonar = 7;
int pinRightSonar = 8;
int pinFrontSonar = 10;
int pinBackSonar = 11;
int pinUpSonar = 12;
int pinDownSonar = 13;

int pinVoltageSensor = A1;
int pinTemperatureSensor = 4;
int pinLED = 13;

//VALUES
int vSonars[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float vVSpeed = 0;
float vHSpeed = 0;
int vDegrees = 0;

float vBatteryVoltage;
float R1VoltageSensor = 30000.0;
float R2VoltageSensor = 7500.0;

int vPressure = 0;

//VALUES COMMAND
String command = "";

int cMotor = 0;
int cXAxis = 0;
int cYAxis = 0;
int cDegrees = 0;

int controlMode = 0;

int calibrateMotors[4] = {0, 0, 0, 0};
float thrustMotors[4] = {0, 0, 0, 0};

int motorNumberTest = -1;

float pitchAccel[2] = {0, 0};
float rollAccel[2] = {0, 0};

int maxDistanceApproach = 20 + 15;

int cCommand = 0;
int lastCCommand = 0;

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

bool isStabilizing = false;

bool isSleeping = false;
bool isSleepingDemand = true;

bool isTestingMotor = false;

bool useSonars = false;
bool useAccelerometer = false;
bool useCompass = false;
bool useSafety = true;

bool debugMode = false;

ThreadController controller = ThreadController();
Thread* outputInformations = new Thread();
Thread* lowSensors = new Thread();
Thread* lostConnectionTest = new Thread();
Thread* sleepingProcess = new Thread();

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
  if (useAccelerometer) {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
  }

  Serial.begin(9600);

  consolePrint("-INIT- Attach variables");

  pinMode(pinLED, OUTPUT);

  DHT11.attach(pinTemperatureSensor);
  DHT11.read();

  vBatteryVoltage = ((analogRead(pinVoltageSensor) * 5.0) / 1024.0) / (R2VoltageSensor / (R1VoltageSensor + R2VoltageSensor));

  motor1.attach(pinMotor1);
  motor2.attach(pinMotor2);
  motor3.attach(pinMotor3);
  motor4.attach(pinMotor4);

  if (useAccelerometer) {
    consolePrint("-INIT- Accelerometer : YES");

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      
      attachInterrupt(0, dmpDataReady, RISING);
      
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();

      consolePrint("-INIT- Accelerometer, DMP : OK");
    } else {
      consolePrint("-INIT- Accelerometer, DMP : FAILED");
    }
  }
  else {
    consolePrint("-INIT- Accelerometer : NO");
  }

  if (useCompass) {
    consolePrint("-INIT- Compass : YES");

    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_30HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
    compass.setOffset(124, -118);
  }
  else {
    consolePrint("-INIT- Compass : NO");
  }

  consolePrint("-INIT- Define threads");

  outputInformations->setInterval(250);
  outputInformations->onRun(sendInformations);
  controller.add(outputInformations);

  lowSensors->setInterval(60000);
  lowSensors->onRun(analyzeLowSensors);
  controller.add(lowSensors);

  lostConnectionTest->setInterval(500);
  lostConnectionTest->onRun(lostConnectionTime);
  controller.add(lostConnectionTest);

  sleepingProcess->setInterval(200);
  sleepingProcess->onRun(sleepDrone);

  consolePrint("-INIT- ENDED");
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

  if (isSleeping || isSleepingDemand)
    digitalWrite(pinLED, HIGH);
  else
    digitalWrite(pinLED, LOW);

  if (useAccelerometer)
    definePitchRoll();

  if (useCompass)
    defineDegrees();

  thrustMotors[0] = 0;
  thrustMotors[1] = 0;
  thrustMotors[2] = 0;
  thrustMotors[3] = 0;

  if (cMotor > 0) {
    if (useCompass)
      setDegrees();

    if (controlMode == 1 && useAccelerometer && useCompass) {
      automaticAxis();
    }
    else
    {
      manualAxis();
    }

    thrustMotors[0] += calibrateMotors[0];
    thrustMotors[1] += calibrateMotors[1];
    thrustMotors[2] += calibrateMotors[2];
    thrustMotors[3] += calibrateMotors[3];

    thrustMotors[0] += cMotor;
    thrustMotors[1] += cMotor;
    thrustMotors[2] += cMotor;
    thrustMotors[3] += cMotor;

    if (thrustMotors[0] < 0 || thrustMotors[1] < 0 || thrustMotors[2] < 0 || thrustMotors[3] < 0) {
      thrustMotors[0] = thrustMotors[1] = thrustMotors[2] = thrustMotors[3] = 0;
    }
  }
  else {
    if (!isSleeping && !isSleepingDemand && isTestingMotor)
      thrustMotors[motorNumberTest] = 5;
  }
  
  motor1.writeMicroseconds(map(thrustMotors[0], 0, 100, MIN_THRUST, MAX_THRUST));
  motor2.writeMicroseconds(map(thrustMotors[1], 0, 100, MIN_THRUST, MAX_THRUST));
  motor3.writeMicroseconds(map(thrustMotors[2], 0, 100, MIN_THRUST, MAX_THRUST));
  motor4.writeMicroseconds(map(thrustMotors[3], 0, 100, MIN_THRUST, MAX_THRUST));

  controller.run();
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
  consolePrint("-COMMAND- '" + command + "'");

  cCommand++;

  char part1 = command.charAt(0);

  int space1 = command.indexOf(" ");

  String part2 = command.substring(space1 + 1);

  if (part1 == 'P') {
    int comma1 = part2.indexOf("|");
    int comma2 = part2.indexOf("|", comma1 + 1);
    int comma3 = part2.indexOf("|", comma2 + 1);

    if (!isSleeping && !isSleepingDemand) {
      cMotor = part2.substring(0, comma1).toInt();
      
      cDegrees = part2.substring(comma1 + 1, comma2).toInt();

      cXAxis = part2.substring(comma2 + 1, comma3).toInt();
      cYAxis = part2.substring(comma3 + 1).toInt();
    }
  }
  else if (part1 == 'M') {
    if (part2.toInt() == 1) {
      controlMode = 1;
    }
    else {
      controlMode = 0;
    }
  }
  else if (part1 == 'B') {
    int comma1 = part2.indexOf("|");
    int comma2 = part2.indexOf("|", comma1 + 1);
    int comma3 = part2.indexOf("|", comma2 + 1);

    int calibrateMotor1 = part2.substring(0, comma1).toInt();
    int calibrateMotor2 = part2.substring(comma1 + 1, comma2).toInt();
    int calibrateMotor3 = part2.substring(comma2 + 1, comma3).toInt();
    int calibrateMotor4 = part2.substring(comma3 + 1).toInt();

    if (calibrateMotor1 >= 0)
      calibrateMotors[0] = calibrateMotor1;

    if (calibrateMotor2 >= 0)
      calibrateMotors[1] = calibrateMotor2;

    if (calibrateMotor3 >= 0)
      calibrateMotors[2] = calibrateMotor3;

    if (calibrateMotor4 >= 0)
      calibrateMotors[3] = calibrateMotor4;
  }
  else if (part1 == 'A') {
    if (part2.equalsIgnoreCase("Y")) {
      CALIBRATE_ACCEL_PITCH = -pitchAccel[0];
      CALIBRATE_ACCEL_ROLL = -rollAccel[0];
    }
  }
  else if (part1 == 'O') {
    int comma1 = part2.indexOf("|");

    int tempAxisSensibility = part2.substring(0, comma1).toInt();
    int tempRotationSensibility = part2.substring(comma1 + 1).toInt();

    if (tempAxisSensibility >= 5) {
      axisSensibility = tempAxisSensibility;
    }

    if (tempRotationSensibility >= 0) {
      rotationSensibility = tempRotationSensibility;
    }
  }
  else if (part1 == 'L') {
    if (part2.equalsIgnoreCase("Y")) {
      isSleepingDemand = true;
    }
    else {
      isSleepingDemand = false;

      informationCommand = true;
      countSendCommand = 0;
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
  else if (part1 == 'T') {
    if (isTestingMotor) {
      isTestingMotor = false;
    }
    else {
      int motorNumber = part2.toInt();

      if (motorNumber > 0 && motorNumber < 5) {
        motorNumberTest = motorNumber - 1;
        isTestingMotor = true;
      }
    }
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
        thrustMotors[2] += rotationSensibility;
        thrustMotors[3] -= rotationSensibility;
      }
      else {
        thrustMotors[0] -= rotationSensibility;
        thrustMotors[1] += rotationSensibility;
        thrustMotors[2] -= rotationSensibility;
        thrustMotors[3] += rotationSensibility;
      }
    }
    else if (cDegrees < vDegrees)
    {
      if ((vDegrees - cDegrees) > ((360 - vDegrees) + cDegrees)) {
        thrustMotors[0] += rotationSensibility;
        thrustMotors[1] -= rotationSensibility;
        thrustMotors[2] += rotationSensibility;
        thrustMotors[3] -= rotationSensibility;
      }
      else {
        thrustMotors[0] -= rotationSensibility;
        thrustMotors[1] += rotationSensibility;
        thrustMotors[2] -= rotationSensibility;
        thrustMotors[3] += rotationSensibility;
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
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    rollAccel[0] = (ypr[1] * 180 / M_PI);
    pitchAccel[0] = (ypr[2] * 180 / M_PI);

    pitchAccel[1] = pitchAccel[0] + CALIBRATE_ACCEL_PITCH;
    rollAccel[1] = rollAccel[0] + CALIBRATE_ACCEL_ROLL;
  }
}

void automaticAxis() {
  if (cYAxis < 0 || cYAxis > 0) {
    if (cYAxis > 0) {
      if (cYAxis > -rollAccel[1]) {
        thrustMotors[2] += 5;
        thrustMotors[3] += 5;
      }
      else {
        thrustMotors[0] += 5;
        thrustMotors[1] += 5;
      }
    }
    else {
      if (-cYAxis > rollAccel[1]) {
        thrustMotors[0] += 5;
        thrustMotors[1] += 5;
      }
      else {
        thrustMotors[2] += 5;
        thrustMotors[3] += 5;
      }
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

  if (cXAxis < 0 || cXAxis > 0) {
    if (cXAxis > 0) {
      if (cXAxis > -pitchAccel[1]) {
        thrustMotors[0] += 5;
        thrustMotors[2] += 5;
      }
      else {
        thrustMotors[1] += 5;
        thrustMotors[3] += 5;
      }
    }
    else {
      if (-cXAxis > pitchAccel[1]) {
        thrustMotors[1] += 5;
        thrustMotors[3] += 5;
      }
      else {
        thrustMotors[0] += 5;
        thrustMotors[2] += 5;
      }
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
    thrustMotors[1] += (((float)cXAxis / (float)45) * axisSensibility);
  }
  else if (cXAxis < 0) {
    thrustMotors[2] += (((float) - cXAxis / (float)45) * axisSensibility);
    thrustMotors[3] += (((float) - cXAxis / (float)45) * axisSensibility);
  }

  if (cYAxis > 0) {
    thrustMotors[1] += (((float)cYAxis / (float)45) * axisSensibility);
    thrustMotors[2] += (((float)cYAxis / (float)45) * axisSensibility);
  }
  else if (cYAxis < 0) {
    thrustMotors[0] += (((float) - cYAxis / (float)45) * axisSensibility);
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
  consolePrint("-THREAD- analyzeLowSensors");

  DHT11.read();

  vBatteryVoltage = ((analogRead(pinVoltageSensor) * 5.0) / 1024.0) / (R2VoltageSensor / (R1VoltageSensor + R2VoltageSensor));
}

void lostConnectionTime() {
  if (useSafety) {
    if (cCommand == lastCCommand) {
      isSleeping = true;
    }
    else {
      isSleeping = false;
    }

    lastCCommand = cCommand;
  }
  else {
    isSleeping = false;
  }
}

void sendInformations() {
  if (informationCommand) {
    if (countSendCommand > 1) {
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
      Serial.print(calibrateMotors[0]);
      Serial.print("|");
      Serial.print(calibrateMotors[1]);
      Serial.print("|");
      Serial.print(calibrateMotors[2]);
      Serial.print("|");
      Serial.println(calibrateMotors[3]);

      countSendCommand++;
    }
  }
  else {
    Serial.print("D D ");
    //Serial.print(vSonars[0][0]);
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
    Serial.print(vBatteryVoltage);
    Serial.print("|");
    Serial.print("0");
    Serial.print("|");
    Serial.print(DHT11.temperature);
    Serial.print("|");
    Serial.print(DHT11.humidity);
    Serial.println();
  }
}

void consolePrint(String message) {
  if (debugMode)
    Serial.println(message);
}




