#include <Servo.h>
#include <drive_controller_msgs/msg/tank.h>

class tankInterface{
    public:
        tankInterface(unsigned leftServoPin, unsigned rightServoPin) {
            left_drive.attach(leftServoPin);
            right_drive.attach(rightServoPin);
        }
        //take message input: -1.0 to 1.0, float
        //write to motor controllers (0-179, map to 53-179)
        void update(const drive_controller_msgs__msg__Tank * msgin) {
            unsigned left_speed = (unsigned)float_map(msgin->left, -1.0, 1.0, 53, 179);
            unsigned right_speed = (unsigned)float_map(msgin->right, -1.0, 1.0, 53, 179);
            left_drive.write(left_speed);
            right_drive.write(right_speed);
        }
        //implement map function using floats for tank message input
        //see here: https://www.arduino.cc/reference/en/language/functions/math/map/
        float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
    private:
        Servo left_drive;
        Servo right_drive;
};

