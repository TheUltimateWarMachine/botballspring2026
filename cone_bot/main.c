#include <kipr/wombat.h>
#include <math.h>
#include <stdlib.h>

#define CLAW 3
#define ARM 1
#define MOTORl 0
#define MOTORr 3
#define SENSORl 3
#define SENSORr 0
#define BLACK 3200
#define RIGHT_MOTOR_CORRECTION 1.002
#define MOTOR_SPEED_250 250
#define MOTOR_SPEED_500 500
#define MOTOR_SPEED_1000 1000
#define MOTOR_SPEED_1500 1500
#define WHITE 2600
#define ARM_DOWN 1760
#define DEGREES 5
#define CLAW_OPEN 750
#define CLAW_POMS 550
#define CLAW_START 920
#define CLAW_CLOSE 1300
#define ARM_CARRY 1600
#define ARM_MID 1200
#define ARM_UP 100
#define ARM_START 900
void move_arm(int end_pos) {
    int current_pos = get_servo_position(ARM);
    printf("move_arm(S): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
    while (current_pos != end_pos) {
        if (current_pos > end_pos) {
            current_pos -= 10;
            if(current_pos < end_pos) {
                current_pos = end_pos;
            }   
        } else {
            current_pos += 10;
            if(current_pos > end_pos) {
                current_pos = end_pos;
            }   
        }
        // printf("Updated: current_pos: %d\n", current_pos);
        set_servo_position(ARM, current_pos);
        msleep(15);
    }
    printf("move_arm(End): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
}
void move_claw(int end_pos) {
    int current_pos = get_servo_position(CLAW);
    printf("move_claw(S): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
    while (current_pos != end_pos) {
        if (current_pos > end_pos) {
            current_pos -= 30;
            if(current_pos < end_pos) {
                current_pos = end_pos;
            }   
        } else {
            current_pos += 30;
            if(current_pos > end_pos) {
                current_pos = end_pos;
            }   
        }
        // printf("Updated: current_pos: %d\n", current_pos);
        set_servo_position(CLAW, current_pos);
        msleep(60);
    }
    printf("move_claw(End): current_pos= %d, end_pos= %d\n", current_pos, end_pos);
}

void move_arm_fast_slow_down(int position) {        
    set_servo_position(ARM,position-500);
    msleep(100);
    move_arm(position);
    msleep(100);
}  

void drive(int speed1,int speed2) {
    mav(MOTORl, speed1);
    mav(MOTORr,speed2* RIGHT_MOTOR_CORRECTION);
    printf("Motor Correction Happening");
}
void drive_till_blk(){
    while (analog(SENSORl) <= BLACK || analog(SENSORr) <= BLACK) {
        drive(MOTOR_SPEED_1500, MOTOR_SPEED_1500);
    }
}
void drive_till_blk_speed(int speed){
    while (analog(SENSORl) <= BLACK || analog(SENSORr) <= BLACK) {
        drive(speed, speed);
    }
}
void drive_till_white(){
    while (analog(SENSORl) >= WHITE || analog(SENSORr) >= WHITE) {
        drive(MOTOR_SPEED_1000,MOTOR_SPEED_1000);
    }
    
}
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
void right_10() {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    while(gmpc(MOTORl) > -100 && gmpc(MOTORr) > -100){
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void right_degrees(int degrees) {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    int ticks = -11.0 * degrees;
    while(gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks){
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void left_degrees(int degrees) {
    printf("RIGHT TURN");
    cmpc(MOTORl);
    cmpc(MOTORr);
    int ticks = -11.28 * degrees;
    while(gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks){
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void back_till_blk(){
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(-MOTOR_SPEED_1500,-MOTOR_SPEED_1500);
    }
}
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
            drive(high, high);   // both on black, go straight
        } else if (sl >= BLACK) {
            drive(low, high);    // flipped from before
        } else if (sr >= BLACK) {
            drive(high, low);    // flipped from before
        } else {
            drive(high, high);   // lost line, go straight
        }
        msleep(1);
    }

    ao();
    msleep(50);
}

void left_turn() {
    cmpc(MOTORl);
    cmpc(MOTORr);
    while(gmpc(MOTORl) < 1000 && gmpc(MOTORr) < 1000) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void left_60() {
    cmpc(MOTORl);
    cmpc(MOTORr);
    while(gmpc(MOTORl) < 700 && gmpc(MOTORr) < 700) {
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void line_follow_v2(int distance, int speed) {
    int high = speed;
    int low = 0.6 * speed;

    cmpc(MOTORl);
    cmpc(MOTORr);
    msleep(100);
    printf("reverse_line_follow(): distance=%d, speed=%d\n", distance, speed);

    while (abs(gmpc(MOTORl)) < distance || abs(gmpc(MOTORr)) < distance) {
        int sl = analog(SENSORl);
        int sr = analog(SENSORr);

        if (sl >= BLACK && sr >= BLACK) {
            drive(high, high);   // both on black, go straight
        } else if (sl >= BLACK) {
            drive(low, high);    // flipped from before
        } else if (sr >= BLACK) {
            drive(high, low);    // flipped from before
        } else {
            drive(high, high);   // lost line, go straight
        }
        msleep(1);
    }

    ao();
    msleep(50);
}

void line_follow(int distance, int speed) {
    //follow black line for set distance.
    int high = speed;
    int low = 0.6 * speed;
    cmpc(MOTORl);
    cmpc(MOTORr);
    printf("line_follow(): distance= %d, speed= %d\n", distance, speed);
    while (gmpc(MOTORl) < distance && gmpc(MOTORr) < distance) {
        msleep(1);
        // if both sensors do not detect black, drive forward
        // if the left sensor detects black, turn left
        // if the right sensor detects black, turn right
        if (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {  
            drive(high, high);   
        } else if (analog(SENSORl) >= BLACK) {
            drive(low, high);
        } else if(analog(SENSORr) >= BLACK) {
            drive(high, low);        
        }
    }
    drive(0, -10);    
    msleep(50);
    ao();
    msleep(50);
}





void go_up_ramp() {
    drive(500,500);
    msleep(700);
    left_degrees(5);
    line_follow(4500,MOTOR_SPEED_1500);
    drive(MOTOR_SPEED_1500,MOTOR_SPEED_1500);
    msleep(2000);
    line_follow(8000,MOTOR_SPEED_1500);
    drive_till_white();
    ao();
}
void move_pick_cone1() {
    printf("starting to go to cone1");
    enable_servos();
    move_claw(CLAW_POMS);
	move_arm_fast_slow_down(ARM_DOWN);
    drive_till_blk();
    drive(MOTOR_SPEED_1000,MOTOR_SPEED_1000);
    msleep(500);
    drive_till_blk();
    ao();
    move_arm(ARM_START);
    msleep(500);
    drive(1050,1050);
    msleep(1700);
    drive(-500,-500);
    msleep(800);
    right_degrees(105);
    drive(-1500,-1500);
    msleep(1000);
    ao();
    right_degrees(25);
    move_arm_fast_slow_down(ARM_DOWN);
    left_degrees(25);

    line_follow(4350,1500);
    left_degrees(40);
    move_arm(ARM_MID);

    right_degrees(30);
    line_follow(850,1500);
    right_degrees(DEGREES);
    printf("Stopped to Pick Cone1");
    move_arm(ARM_DOWN);
    msleep(300);
    left_degrees(DEGREES);
    msleep(1000);
    set_servo_position(CLAW,710);
    printf("Closed Claw, Ended Cone1");
}   
void move_pick_cone2() {
    line_follow(5300, MOTOR_SPEED_1500);
    right_degrees(DEGREES);
    drive(1500,1500);
    msleep(1200);
    ao();
    set_servo_position(CLAW, CLAW_CLOSE);
    left_degrees(DEGREES);
    printf("Closed Claw, Ended Cone1");
    move_arm(ARM_UP);
}


void return_cone()
{/*
    right_degrees(215);
    line_follow(9800,1500);
    left_turn();
    drive_till_blk();
    drive(1000,1000);
    msleep(3300);
    left_degrees(70);
    */
    go_up_ramp();
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(800);
    right_degrees(25);
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(2700);
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(1000);
    back_till_blk();
	left_degrees(40);
    ao();
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(300);
    ao();
    move_arm_fast_slow_down(ARM_DOWN);
    msleep(500);
    move_claw(CLAW_POMS);
    msleep(500);
    move_arm(ARM_UP);
    msleep(500);
    move_claw(CLAW_CLOSE);
    msleep(500);
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(500);
    back_till_blk();
    right_degrees(15);
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(4000);
    left_degrees(120);
    drive(-MOTOR_SPEED_500,-MOTOR_SPEED_500);
    msleep(1900);
    right_degrees(30);
    
    
    
}


int main() {
    enable_servos();
    int start_time = seconds();
    printf("Start Time: %d", start_time);
    
    //move_pick_cone1();
    //move_pick_cone2();
    return_cone();
    /*
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(1000);
    ao();
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(1000);
    ao();
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(1000);
    ao();
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(1000);
    ao();
    drive(MOTOR_SPEED_500,MOTOR_SPEED_500);
    msleep(1000);
    ao();
    */
    int end_time = seconds();
    int total_time = end_time - start_time;
    printf("End Time: %d", end_time);
    printf("Total Time: %d",total_time);
    
    
    /*
    drive_till_blk_speed(MOTOR_SPEED_500);
    ao();
    msleep(1000);
    drive(500,500);
    msleep(7300);
    left_degrees(75);
   
    left_degrees(30);
    go_up_ramp();
    left_degrees(10);
    drive(-500,-500);
    msleep(750);
    ao();
    move_arm_fast_slow_down(ARM_DOWN);
    msleep(500);
    move_claw(500);
    msleep(500);
    set_servo_position(ARM, 0);
    ao();
    right_degrees(30);
    set_servo_position(CLAW,CLAW_CLOSE);
    drive(1000,1000);
    msleep(2425);
    set_servo_position(CLAW,ARM_START);
    ao();
    
    right_degrees(215);
    line_follow(5000,1500);
    */

    
 }


