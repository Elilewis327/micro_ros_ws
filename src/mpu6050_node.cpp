#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3_stamped.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <functional>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include "I2Cdev.h"
//#include "MPU6050.h"

#define MPU_FRAMEID "base_imu"

class mpu6050_node {

    
    const unsigned int timer_delay = 20;

    //MPU6050 mpu;

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

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    
    // orientation/motion vars
    double q[4];            // [w, x, y, z]         quaternion container
    uint16_t aa[3];         // [x, y, z]            accel sensor measurements
    uint16_t aaReal[3];     // [x, y, z]            gravity-free accel sensor measurements
    uint16_t aaWorld[3];    // [x, y, z]            world-frame accel sensor measurements
    float gravity[3];       // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    int sample_rate = 1000 / timer_delay;

    int ax, ay, az, gx, gy, gz;

    bool debug = false;

    double angular_velocity_covariance, pitch_roll_covariance, yaw_covariance, linear_acceleration_covariance;
    double linear_acceleration_stdev_, angular_velocity_stdev_, yaw_stdev_, pitch_roll_stdev_;

    

  public:
    mpu6050_node() {

        

        

        // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
        linear_acceleration_stdev_ = (400 / 1000000.0) * 9.807;
    };
};
