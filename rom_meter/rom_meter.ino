#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class IMU {
  MPU6050 mpu;
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  bool dmpReady = false;

  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  public:
  Quaternion q;           // [w, x, y, z]         quaternion container
  IMU(uint8_t addr);
  void init();
  void update();
  VectorFloat get_nose_vector();
};

IMU::IMU(uint8_t addr) {
  mpu = MPU6050(addr);
}

void IMU::init() {
  mpu.initialize();
  //mpu.testConnection();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      mpu.setDMPEnabled(true);
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void IMU::update() {
  if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
    }
}

VectorFloat IMU::get_nose_vector() {
  VectorFloat n_vec(1.0, 0.0, 0.0);
  return n_vec.getRotated(&q);
}


IMU wrist(0x69), finger(0x68);

VectorFloat wrist_vec, finger_vec;

void setup() {
  // put your setup code here, to run once:
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  finger.init();
  wrist.init();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  finger.update();
  wrist.update();
  wrist_vec = wrist.get_nose_vector();
  finger_vec = finger.get_nose_vector();
  float dot_prod = 
    wrist_vec.x * finger_vec.x +
    wrist_vec.y * finger_vec.y +
    wrist_vec.z * finger_vec.z;
  float theta = acos(dot_prod / (wrist_vec.getMagnitude() * finger_vec.getMagnitude())) * 180.0 / PI;
  Serial.println(theta);
}

