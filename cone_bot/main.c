#include <kipr/wombat.h>
#include <math.h>
#include <stdlib.h>

// Servo port assignments
#define CLAW 1
#define ARM 0

// Motor port assignments
#define MOTORl 0
#define MOTORr 3

// Analog sensor port assignments
#define SENSORl 0
#define SENSORr 3

// Sensor threshold values
#define BLACK 3000   // analog value at or above this = black line detected
#define WHITE 2500   // analog value at or below this = white surface detected

// Right motor multiplier to correct for hardware imbalance
#define RIGHT_MOTOR_CORRECTION 1

// Named motor speed constants
#define MOTOR_SPEED_500  500
#define MOTOR_SPEED_1000 1000
#define MOTOR_SPEED_1500 1500


// Smoothly moves the arm servo to end_pos in steps of 10
void move_arm(int end_pos) {
    int current_pos = get_servo_position(ARM);
    printf("move_arm(S): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
    while (current_pos != end_pos) {
        if (current_pos > end_pos) {
            current_pos -= 10;
            if (current_pos < end_pos) current_pos = end_pos;
        } else {
            current_pos += 10;
            if (current_pos > end_pos) current_pos = end_pos;
        }
        set_servo_position(ARM, current_pos);
        msleep(30);
    }
    printf("move_arm(End): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
}

// Smoothly moves the claw servo to end_pos in steps of 30
void move_claw(int end_pos) {
    int current_pos = get_servo_position(CLAW);
    printf("move_claw(S): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
    while (current_pos != end_pos) {
        if (current_pos > end_pos) {
            current_pos -= 30;
            if (current_pos < end_pos) current_pos = end_pos;
        } else {
            current_pos += 30;
            if (current_pos > end_pos) current_pos = end_pos;
        }
        set_servo_position(CLAW, current_pos);
        msleep(60);
    }
    printf("move_claw(End): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
}


// Sets both motors; applies right motor correction to compensate for drift
void drive(int speed1, int speed2) {
    mav(MOTORl, speed1);
    mav(MOTORr, speed2 * RIGHT_MOTOR_CORRECTION);
    printf("Motor Correction Happening");
}

// Drives forward until either sensor detects a black line
void drive_till_blk() {
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(1500, 1500);
    }
}

// Drives forward until both sensors leave the black line (detect white)
void drive_till_white() {
    while (analog(SENSORl) >= WHITE && analog(SENSORr) >= WHITE) {
        drive(1500, 1500);
    }
}

// Turns right ~90 deg using encoder counts (~1000 ticks)
void right_turn() {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    while (gmpc(MOTORl) > -1000 && gmpc(MOTORr) > -1000) {
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}

// Small right nudge (~100 ticks)
void right_10() {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    while (gmpc(MOTORl) > -100 && gmpc(MOTORr) > -100) {
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}

// Turns right by a specified number of degrees (11.11 ticks/degree)
void right_degrees(int degrees) {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    int ticks = -11.11 * degrees;
    while (gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks) {
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}

// Turns left by a specified number of degrees (11.11 ticks/degree)
void left_degrees(int degrees) {
    printf("LEFT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    int ticks = -11.11 * degrees;
    while (gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}

// Reverses until either sensor detects a black line
void back_till_blk() {
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(-1500, -1500);
    }
}

// Follows a black line in reverse for a given encoder distance at a given speed
void reverse_line_follow(int distance, int speed) {
    int high = -speed;
    int low = (int)(-0.6 * speed);

    cmpc(MOTORl);
    cmpc(MOTORr);
    msleep(100);
    printf("reverse_line_follow(): distance=%d, speed=%d\n", distance, speed);

    while (abs(gmpc(MOTORl)) < distance || abs(gmpc(MOTORr)) < distance) {
        int sl = analog(SENSORl);
        int sr = analog(SENSORr);

        if (sl >= BLACK && sr >= BLACK) {
            drive(high, high);   // both on black — go straight back
        } else if (sl >= BLACK) {
            drive(low, high);    // left on black — correct right
        } else if (sr >= BLACK) {
            drive(high, low);    // right on black — correct left
        } else {
            drive(high, high);   // lost line — keep going straight
        }
        msleep(1);
    }

    ao();
    msleep(50);
}

// Turns left ~90 deg using encoder counts (~1000 ticks)
void left_turn() {
    cmpc(MOTORl);
    cmpc(MOTORr);
    while (gmpc(MOTORl) < 1000 && gmpc(MOTORr) < 1000) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}

// Shorter left turn (~60 deg, 700 ticks)
void left_60() {
    cmpc(MOTORl);
    cmpc(MOTORr);
    while (gmpc(MOTORl) < 700 && gmpc(MOTORr) < 700) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}

// Follows a black line forward for a given encoder distance at a given speed.
// Steers by slowing the inner motor when a sensor detects the line edge.
void line_follow(int distance, int speed) {
    int high = speed;
    int low = 0.6 * speed;
    cmpc(MOTORl);
    cmpc(MOTORr);
    printf("line_follow(): distance= %d, speed= %d\n", distance, speed);
    while (gmpc(MOTORl) < distance && gmpc(MOTORr) < distance) {
        msleep(1);
        if (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
            drive(high, high);        // both off line — go straight
        } else if (analog(SENSORl) >= BLACK) {
            drive(low, high);         // left sensor on line — steer left
        } else if (analog(SENSORr) >= BLACK) {
            drive(high, low);         // right sensor on line — steer right
        }
    }
    drive(0, -10);   // brief brake to stop cleanly
    msleep(50);
    ao();
    msleep(50);
}


// Drives forward until black, then line-follows up the ramp, then stops at white
void go_up_ramp() {
    drive_till_blk();
    line_follow(14000, 1500);
    drive_till_white();
    ao();
}

// Navigates to cone 1, lowers arm/claw to pick position, then closes claw to grab
void move_pick_cone1() {
    printf("starting to go to cone1");
    enable_servos();
    move_arm(750);       // lower arm to approach height
    move_claw(400);      // open claw
    drive_till_blk();
    drive(1000, 1000);
    msleep(500);
    drive_till_blk();
    drive(1050, 1050);
    msleep(1500);
    right_turn();
    line_follow(4000, 1500);
    ao();
    right_degrees(5);    // fine alignment adjustment
    printf("Stopped to Pick Cone1");
    move_arm(1848);      // drop arm onto cone
    move_claw(1667);     // close claw to grip cone
    printf("Closed Claw, Ended Cone1");
}

// Navigates to cone 2, repositions arm/claw to pick it up
void move_pick_cone2() {
    enable_servos();
    line_follow(4000, 1500);
    ao();
    printf("Stopped to Pick Cone1");
    move_claw(700);       // partially open claw
    move_arm(1200);       // raise arm to intermediate position
    line_follow(1800, 1500);
    drive(-1000, -1000);  // back up slightly to align
    msleep(600);
    ao();
    right_degrees(10);    // adjust angle for pickup
    move_arm(1848);       // lower arm onto cone
    left_degrees(10);     // correct back
    drive(-1000, -1000);
    msleep(300);
    ao();
    move_claw(1667);      // close claw to grip cone
    move_arm(400);        // lift arm to carry position
    printf("Closed Claw, Ended Cone1");
}

// Carries cone to drop zone: line-follows, turns, drives to platform, goes up ramp, releases cone
void drop_cone() {
    line_follow(5300, 1500);
    left_turn();
    drive_till_blk();
    drive(1000, 1000);
    msleep(3700);         // drive across open floor to platform base
    left_60();
    go_up_ramp();
    move_arm(1800);       // lower arm to release height
    move_claw(500);       // open claw to drop cone
    right_degrees(50);    // turn away after drop
}

// Returns robot to starting area after dropping a cone
void return_cone() {
    move_arm(0);          // raise arm to travel position
    right_turn();
    right_degrees(80);
    line_follow(9000, 1500);
    left_turn();
    drive_till_blk();
    drive(1000, 1000);
    msleep(3400);
    left_degrees(45);
    go_up_ramp();
    drive(1000, 1000);
    msleep(1800);
    left_degrees(77);
    drive(-1000, -1000);  // back into final position
    msleep(1300);
    ao();
    move_arm(1800);
    move_claw(500);       // open claw
    move_arm(600);        // settle arm to resting position
}


int main() {
    move_pick_cone1();   // pick up first cone
    move_pick_cone2();   // pick up second cone
    return_cone();       // return to start / place cone
    return 0;
}
