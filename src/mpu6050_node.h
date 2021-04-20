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
class mpu6050_node {

    bool ado = false;

    MPU6050 mpu;

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t fifoBuffer[64]; // FIFO storage buffer
   
    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    int ax, ay, az, gx, gy, gz;

    bool debug = false;

    double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
    double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;

  public:
    mpu6050_node(): ax(0), ay(0), az(1788), gx(220), gy(76), gz(-85) {
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

        // verify connection - usually fails, but still receives data back
        Serial.println("Testing device connections...");
        Serial.print(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

        // load and configure the DMP
        Serial.print("Initializing DMP...\n");
        uint8_t devStatus = mpu.dmpInitialize();

        // Set accel offsets.
        Serial.print("Setting X accel offset: ");
        Serial.println(ax);
        mpu.setXAccelOffset(ax);
        Serial.print("Setting Y accel offset: ");
        Serial.println(ay);
        mpu.setYAccelOffset(ay);
        Serial.print("Setting Z accel offset: ");
        Serial.println(az);
        mpu.setZAccelOffset(az);

        // Set gyro offsets.
        Serial.print("Setting X gyro offset: ");
        Serial.println(gx);
        mpu.setXGyroOffset(gx);
        Serial.print("Setting Y gyro offset: ");
        Serial.println(gy);
        mpu.setYGyroOffset(gy);
        Serial.print("Setting Z gyro offset: ");
        Serial.println(gz);
        mpu.setZGyroOffset(gz);

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            // mpu.CalibrateAccel(6);
            // mpu.CalibrateGyro(6);
            // mpu.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.print("Enabling DMP...\n");
            mpu.setDMPEnabled(true);

            // set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.print("DMP ready!\n");
            dmpReady = true;
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print("DMP Initialization failed: ");
            Serial.println(devStatus);
        }

    };

    void update(sensor_msgs__msg__Imu& imu_msg) {
        if (!dmpReady) return;

        // read a packet from FIFO
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

            // Get important data
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

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

            // display Euler angles in degrees
            // Should be in rad/sec.
            imu_msg.angular_velocity.x = ypr[2];
            imu_msg.angular_velocity.y = ypr[1];
            imu_msg.angular_velocity.z = ypr[0];

            // display real acceleration, adjusted to remove gravity
            // https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps20.h               

            // By default, accel is in arbitrary units with a scale of 16384/1g.
            // Per http://www.ros.org/reps/rep-0103.html
            // and http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
            // should be in m/s^2.
            // 1g = 9.80665 m/s^2, so we go arbitrary -> g -> m/s^s
            imu_msg.linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
            imu_msg.linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
            imu_msg.linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;

            if(debug) {
                // Print orientation
                Serial.print("update: \n\torientation = (");
                Serial.print(q.x);
                Serial.print(", ");
                Serial.print(q.y);
                Serial.print(", ");
                Serial.print(q.z);
                Serial.print(", ");
                Serial.print(q.w);  
                Serial.print(") \n\ta_vel = (");

                // Print YPR
                Serial.print(ypr[0]);
                Serial.print(", ");
                Serial.print(ypr[1]);
                Serial.print(", ");
                Serial.print(ypr[2]);  
                Serial.print(") \n\tl_accel = (");

                // Print acceleration
                Serial.print(aaReal.x * 1/16384. * 9.80665);
                Serial.print(", ");
                Serial.print(aaReal.y * 1/16384. * 9.80665);
                Serial.print(", ");
                Serial.print(aaReal.z * 1/16384. * 9.80665);  
                Serial.print(") \n\n");

                int16_t AX, AY, AZ, GX, GY, GZ;
                mpu.getMotion6(&AX, &AY, &AZ, &GX, &GY, &GZ);
                Serial.print("a/g:\t");
                Serial.print(AX); Serial.print("\t");
                Serial.print(AY); Serial.print("\t");
                Serial.print(AZ); Serial.print("\t");
                Serial.print(GX); Serial.print("\t");
                Serial.print(GY); Serial.print("\t");
                Serial.println(GZ);
            }
        }
    }
};
