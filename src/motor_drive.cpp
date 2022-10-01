#include "motor_drive.h"
#include "robot_constant.h"
#include "pin_map.h"

#define SERVOMIN  60 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  520 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;



// See this lib: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all
//pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);  for ease of use

void init_motors() {

    pwm.begin();

    Serial.println("Initting");

    pwm.setOscillatorFrequency(27000000); //set w/ ossicloscope?
    pwm.setPWMFreq(SERVO_FREQ);  
    

}

void command_motors() {

    superior_right_shoulder_abductor_pwm = map(superior_right_shoulder_abductor_pos, -1.01, 1.176, 160, 440) + superior_right_shoulder_abductor_offset;
    superior_left_shoulder_abductor_pwm = map(superior_left_shoulder_abductor_pos, -1.01, 1.176, 420, 140) + superior_left_shoulder_abductor_offset;
    inferior_right_shoulder_abductor_pwm = map(inferior_right_shoulder_abductor_pos, -1.01, 1.176, 420, 140) + inferior_right_shoulder_abductor_offset;
    inferior_left_shoulder_abductor_pwm = map(inferior_left_shoulder_abductor_pos, -1.01, 1.176, 160, 440) + inferior_left_shoulder_abductor_offset;

    superior_right_arm_extensor_pwm = map(superior_right_arm_extensor_pos, -1.78, 1.61, 500, 100) + superior_right_arm_extensor_offset;
    superior_left_arm_extensor_pwm = map(superior_left_arm_extensor_pos, -1.78, 1.61, 80, 480) + superior_left_arm_extensor_offset;
    inferior_right_arm_extensor_pwm = map(inferior_right_arm_extensor_pos, -1.45, 1.69, 460, 90) + inferior_right_arm_extensor_offset;
    inferior_left_arm_extensor_pwm = map(inferior_left_arm_extensor_pos, -1.45, 1.69, 120, 490) + inferior_left_arm_extensor_offset;

    superior_right_forearm_extensor_pwm = map(superior_right_forearm_extensor_pos, -0.94, 2.72, 180, 610) + superior_right_forearm_extensor_offset;
    superior_left_forearm_extensor_pwm = map(superior_left_forearm_extensor_pos, 0, 2.89, 290, -40) + superior_left_forearm_extensor_offset;
    inferior_right_forearm_extensor_pwm = map(inferior_right_forearm_extensor_pos, -0.26, 2.71, 260, 630) + inferior_right_forearm_extensor_offset;
    inferior_left_forearm_extensor_pwm = map(inferior_left_forearm_extensor_pos, -0.33, 2.46, 330, 0) + inferior_left_forearm_extensor_offset;


    pwm.setPWM(0, 0, superior_right_shoulder_abductor_pwm);
    pwm.setPWM(1, 0, superior_left_shoulder_abductor_pwm);
    pwm.setPWM(2, 0, inferior_right_shoulder_abductor_pwm);
    pwm.setPWM(3, 0, inferior_left_shoulder_abductor_pwm);

    pwm.setPWM(4, 0, superior_right_arm_extensor_pwm);
    pwm.setPWM(5, 0, superior_left_arm_extensor_pwm);
    pwm.setPWM(6, 0, inferior_right_arm_extensor_pwm);
    pwm.setPWM(7, 0, inferior_left_arm_extensor_pwm);

    pwm.setPWM(8, 0, superior_right_forearm_extensor_pwm);
    pwm.setPWM(9, 0, superior_left_forearm_extensor_pwm);
    pwm.setPWM(10, 0, inferior_right_forearm_extensor_pwm);
    pwm.setPWM(11, 0, inferior_left_forearm_extensor_pwm);

}