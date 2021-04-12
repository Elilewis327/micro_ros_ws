#include <sensor_msgs/msg/imu.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <math.h>

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
class mpu6050_node {

    bool ado = false;
    const unsigned int timer_delay = 20;

    MPU6050 mpu;

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    int sample_rate = 1000 / timer_delay;

    int ax, ay, az, gx, gy, gz;

    bool debug = false;

    double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
    double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;

    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady() {
        mpuInterrupt = true;
    }

  public:
    mpu6050_node(): ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {

        // join I2C bus (I2Cdev library doesn't do this automatically)
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
            Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif

        
        mpu.initialize();
        // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
        linear_acceleration_stdev_ = (400 / 1000000.0) * 9.807;
        
        // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
        angular_velocity_stdev_ = 0.05 * (M_PI / 180.0);

        // 1 degree for pitch and roll
        pitch_roll_stdev_ = 1.0 * (M_PI / 180.0);

        // 5 degrees for yaw
        yaw_stdev_ = 5.0 * (M_PI / 180.0);

        angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
        linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
        pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
        yaw_covariance = yaw_stdev_ * yaw_stdev_;


        // verify connection
        Serial.println(F("Testing device connections..."));
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

        // wait for ready
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again

        // load and configure the DMP
        Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // Set accel offsets.
        Serial.println("Setting X accel offset: \n");
        mpu.setXAccelOffset(ax);
        Serial.println("Setting Y accel offset: \n");
        mpu.setYAccelOffset(ay);
        Serial.println("Setting Z accel offset: \n");
        mpu.setZAccelOffset(az);

        // Set gyro offsets.
        Serial.println("Setting X gyro offset: \n");
        mpu.setXGyroOffset(gx);
        Serial.println("Setting Y gyro offset: \n");
        mpu.setYGyroOffset(gy);
        Serial.println("Setting Z gyro offset: \n");
        mpu.setZGyroOffset(gz);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // turn on the DMP, now that it's ready
            Serial.println("Enabling DMP...\n");
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println("DMP ready!\n");
            dmpReady = true;

            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.println("DMP Initialization failed (code %d)\n");
        }

    };

    void update(sensor_msgs__msg__Imu& imu_msg) {
        if (!dmpReady) return;

        fifoCount = mpu.getFIFOCount();

        if (fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            if(debug) Serial.println("FIFO overflow!\n");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (fifoCount >= 42) {

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetQuaternion(&q, fifoBuffer);

            imu_msg.orientation.x = q.x;
            imu_msg.orientation.y = q.y;
            imu_msg.orientation.z = q.z;
		    imu_msg.orientation.w = q.w;

            imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance;
            imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance;
            imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance;
        
            imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance;
            imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance;
            imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance;

            imu_msg.orientation_covariance[0] = pitch_roll_covariance;
            imu_msg.orientation_covariance[4] = pitch_roll_covariance;
            imu_msg.orientation_covariance[8] = yaw_covariance;

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                // Should be in rad/sec.
                imu_msg.angular_velocity.x = ypr[2];
                imu_msg.angular_velocity.y = ypr[1];
                imu_msg.angular_velocity.z = ypr[0];

            #endif

            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                // https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps20.h
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

                // By default, accel is in arbitrary units with a scale of 16384/1g.
                // Per http://www.ros.org/reps/rep-0103.html
                // and http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
                // should be in m/s^2.
                // 1g = 9.80665 m/s^2, so we go arbitrary -> g -> m/s^s
                imu_msg.linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
                imu_msg.linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
                imu_msg.linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;

            #endif

            if(debug) Serial.println("\n");
        }
    }
};
