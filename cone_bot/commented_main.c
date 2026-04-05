#include <kipr/wombat.h>
#include <math.h>
#include <stdlib.h>

// Servo and motor port definitions
#define CLAW 1
#define ARM 0
#define MOTORl 0
#define MOTORr 3

// Sensor ports
#define SENSORl 0
#define SENSORr 3

// Threshold values for line detection
#define BLACK 3000
#define WHITE 2500

// Motor tuning constants
#define RIGHT_MOTOR_CORRECTION 1
#define MOTOR_SPEED_500 500
#define MOTOR_SPEED_1000 1000
#define MOTOR_SPEED_1500 1500

// Smoothly moves the arm servo to a target position
void move_arm(int end_pos) {
    int current_pos = get_servo_position(ARM);
    printf("move_arm(S): current_pos= %d, end_pos= %d\n", current_pos, end_pos);

    // Incrementally adjust servo position for smooth motion
    while (current_pos != end_pos) {
        if (current_pos > end_pos) {
            current_pos -= 10;
            if(current_pos < end_pos) current_pos = end_pos;
        } else {
            current_pos += 10;
            if(current_pos > end_pos) current_pos = end_pos;
        }
        set_servo_position(ARM, current_pos);
        msleep(30);
    }

    printf("move_arm(End): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
}

// Smoothly moves the claw servo to a target position
void move_claw(int end_pos) {
    int current_pos = get_servo_position(CLAW);
    printf("move_claw(S): current_pos= %d, end_pos= %d\n", current_pos, end_pos);

    while (current_pos != end_pos) {
        if (current_pos > end_pos) {
            current_pos -= 30;
            if(current_pos < end_pos) current_pos = end_pos;
        } else {
            current_pos += 30;
            if(current_pos > end_pos) current_pos = end_pos;
        }
        set_servo_position(CLAW, current_pos);
        msleep(60);
    }

    printf("move_claw(End): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
}

// Drives robot with independent left/right speeds
void drive(int speed1,int speed2) {
    mav(MOTORl, speed1);
    mav(MOTORr, speed2 * RIGHT_MOTOR_CORRECTION); // apply correction to right motor
    printf("Motor Correction Happening");
}

// Drive forward until both sensors detect black
void drive_till_blk(){
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(1500,1500);
    }
}

// Drive forward until both sensors detect white
void drive_till_white(){
    while (analog(SENSORl) >= WHITE && analog(SENSORr) >= WHITE) {
        drive(1500,1500);
    }
}

// Perform a right turn using encoder ticks
void right_turn() {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);

    while(gmpc(MOTORl) > -1000 && gmpc(MOTORr) > -1000){
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }

    ao();
    msleep(50);
}

// Small right adjustment
void right_10() {
    cmpc(MOTORl);
    cmpc(MOTORr);

    while(gmpc(MOTORl) > -100 && gmpc(MOTORr) > -100){
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }

    ao();
    msleep(50);
}

// Turn right by specified degrees (approximate conversion to ticks)
void right_degrees(int degrees) {
    cmpc(MOTORl);
    cmpc(MOTORr);

    int ticks = -11.11 * degrees;
    while(gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks){
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }

    ao();
    msleep(50);
}

// Turn left by specified degrees
void left_degrees(int degrees) {
    cmpc(MOTORl);
    cmpc(MOTORr);

    int ticks = -11.11 * degrees;
    while(gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks){
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }

    ao();
    msleep(50);
}

// Reverse until black line is detected
void back_till_blk(){
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(-1500,-1500);
    }
}

// Reverse line following for a given distance
void reverse_line_follow(int distance, int speed) {
    int high = -speed;
    int low = (int)(-0.6 * speed);

    cmpc(MOTORl);
    cmpc(MOTORr);
    msleep(100);

    while (abs(gmpc(MOTORl)) < distance || abs(gmpc(MOTORr)) < distance) {
        int sl = analog(SENSORl);
        int sr = analog(SENSORr);

        // Basic line-follow logic (reversed)
        if (sl >= BLACK && sr >= BLACK) {
            drive(high, high);
        } else if (sl >= BLACK) {
            drive(low, high);
        } else if (sr >= BLACK) {
            drive(high, low);
        } else {
            drive(high, high);
        }
        msleep(1);
    }

    ao();
    msleep(50);
}

// Left turn using encoders
void left_turn() {
    cmpc(MOTORl);
    cmpc(MOTORr);

    while(gmpc(MOTORl) < 1000 && gmpc(MOTORr) < 1000) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }

    ao();
    msleep(50);
}

// Smaller left turn
void left_60() {
    cmpc(MOTORl);
    cmpc(MOTORr);

    while(gmpc(MOTORl) < 700 && gmpc(MOTORr) < 700) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }

    ao();
    msleep(50);
}

// Forward line following for a given distance
void line_follow(int distance, int speed) {
    int high = speed;
    int low = 0.6 * speed;

    cmpc(MOTORl);
    cmpc(MOTORr);

    while (gmpc(MOTORl) < distance && gmpc(MOTORr) < distance) {
        msleep(1);

        // Adjust direction based on sensor readings
        if (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {  
            drive(high, high);
        } else if (analog(SENSORl) >= BLACK) {
            drive(low, high);
        } else if(analog(SENSORr) >= BLACK) {
            drive(high, low);
        }
    }

    drive(0, -10); // slight stop correction
    msleep(50);
    ao();
    msleep(50);
}

// Drives robot up a ramp using line following
void go_up_ramp() {
    drive_till_blk();
    line_follow(14000,1500);
    drive_till_white();
    ao();
}

// Sequence to pick up first cone
void move_pick_cone1() {
    enable_servos();
    move_arm(750);
    move_claw(400);

    drive_till_blk();
    drive(1000,1000);
    msleep(500);

    drive_till_blk();
    drive(1050,1050);
    msleep(1500);

    right_turn();
    line_follow(4000,1500);

    ao();
    right_degrees(5);

    move_arm(1848);
    move_claw(1667); // grab cone
}

// Sequence to pick up second cone
void move_pick_cone2() {
    enable_servos();

    line_follow(4000,1500);
    ao();

    move_claw(700);
    move_arm(1200);

    line_follow(1800,1500);

    drive(-1000,-1000);
    msleep(600);
    ao();

    right_degrees(10);
    move_arm(1848);

    left_degrees(10);

    drive(-1000,-1000);
    msleep(300);
    ao();

    move_claw(1667);
    move_arm(400);
}

// Drops cone at target location
void drop_cone() {
    line_follow(5300,1500);
    left_turn();

    drive_till_blk();

    drive(1000,1000);
    msleep(3700);

    left_60();
    go_up_ramp();

    move_arm(1800);
    move_claw(500); // release

    right_degrees(50);
}

// Returns to starting area after dropping cone
void return_cone() {
    move_arm(0);

    right_turn();
    right_degrees(80);

    line_follow(9000,1500);

    left_turn();
    drive_till_blk();

    drive(1000,1000);
    msleep(3400);

    left_degrees(45);
    go_up_ramp();

    drive(1000,1000);
    msleep(1800);

    left_degrees(77);

    drive(-1000,-1000);
    msleep(1300);
    ao();

    move_arm(1800);
    move_claw(500);
    move_arm(600);
}

// Main program execution
int main() {
    move_pick_cone1();
    move_pick_cone2();
    return_cone();
}
