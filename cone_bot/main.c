#include <kipr/wombat.h>
#include <math.h>
#include <stdlib.h>

#define CLAW 2
#define ARM 3
#define MOTORl 0
#define MOTORr 3
#define SENSORl 3
#define SENSORr 0
#define BLACK 3000
#define RIGHT_MOTOR_CORRECTION 1.002
#define MOTOR_SPEED_250 250
#define MOTOR_SPEED_500 500
#define MOTOR_SPEED_1000 1000
#define MOTOR_SPEED_1500 1500
#define WHITE 2000
#define ARM_DOWN 1930
#define DEGREES 5
#define CLAW_OPEN 550
#define CLAW_POMS 730
#define CLAW_START 920
#define CLAW_CLOSE 1300
#define ARM_CARRY 1600
#define ARM_MID 1200
#define ARM_UP 400
#define ARM_START 900
#define CONE1_LINE_FOLLOW 6000
#define CONE2_LINE_FOLLOW 2000
#define RETURN_LINE_FOLLOW 9100
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
void drive_till_white(int timeout_secs) {
    int start_time =  seconds();
    printf("drive_till_white(START): start_time= %d, timeout_secs= %d\n", start_time, timeout_secs);
    while (analog(SENSORl) >= WHITE || analog(SENSORr) >= WHITE) {
        if ( seconds() - start_time >= timeout_secs) {
            printf("drive_till_white(): TIMEOUT after %d secs, start_time= %d\n", timeout_secs, start_time);
            break;
        }
        drive(MOTOR_SPEED_1000, MOTOR_SPEED_1000);
    }
    printf("drive_till_white(END): start_time= %d, seconds= %d\n", start_time, (int) seconds());
    ao();
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
    int ticks = 11.0 * degrees;
    while(gmpc(MOTORl) < ticks || gmpc(MOTORr) > -ticks){
        drive(MOTOR_SPEED_500, -MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void left_degrees(int degrees) {
    printf("LEFT TURN START\n");
    cmpc(MOTORl);
    cmpc(MOTORr);
    int ticks = 11 * degrees;
    while(gmpc(MOTORl) > -ticks || gmpc(MOTORr) < ticks){
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
    printf("LEFT TURN END: Motor_L= %d, Motor_R= %d\n", gmpc(MOTORl),  gmpc(MOTORr));
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
void line_follow(int distance, int speed) {
    int high = speed;
    int low = 0.6 * speed;

    cmpc(MOTORl);
    cmpc(MOTORr);
    msleep(100);
    printf("line_follow(): distance=%d, speed=%d\n", distance, speed);

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

void line_follow_old(int distance, int speed) {
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
    line_follow(11000, MOTOR_SPEED_1500);
    //drive_till_white(3);
    drive_till_blk();
}

void move_pick_cone1() {
    printf("starting to go to cone1");
    enable_servos();
    move_claw(CLAW_POMS);
    right_degrees(DEGREES);
    drive_till_blk();
    drive(MOTOR_SPEED_1000,MOTOR_SPEED_1000);
    msleep(500);
    drive_till_blk();
    ao();
    left_degrees(DEGREES);
    msleep(500);
    drive(1050,1050);
    msleep(1700);
    drive(-500,-500);
    msleep(1100);
    right_degrees(90);
    //msleep(7000);
    line_follow(CONE1_LINE_FOLLOW,MOTOR_SPEED_1500);
    right_degrees(DEGREES);
    move_arm(ARM_DOWN);
    drive(MOTOR_SPEED_1500,MOTOR_SPEED_1500);
    msleep(500);
    ao();
    left_degrees(DEGREES);
    /*right_degrees(DEGREES);
    printf("Stopped to Pick Cone1");
    move_arm(ARM_DOWN);
    msleep(300);
    left_degrees(DEGREES);
    msleep(1000);
    set_servo_position(CLAW,710);
    printf("Closed Claw, Ended Cone1");*/
}   
void move_pick_cone2() {
    /*
    line_follow(2000, MOTOR_SPEED_1500);
    right_degrees(DEGREES);
    drive(MOTOR_SPEED_1500,MOTOR_SPEED_1500);
    msleep(300);
	ao();
    msleep(500);
    */
    
    line_follow(CONE2_LINE_FOLLOW, MOTOR_SPEED_1500);
    right_degrees(DEGREES);
    drive(MOTOR_SPEED_1500, MOTOR_SPEED_1500);
    msleep(1000);
    ao();
    msleep(200);
    msleep(200);
    set_servo_position(CLAW, CLAW_CLOSE);
    left_degrees(DEGREES);
    msleep(200);
    printf("Closed Claw, Ended Cone1");
    move_arm(ARM_UP);
    msleep(200);
}

void return_cone()
{
    drive(-MOTOR_SPEED_1500,-MOTOR_SPEED_1500);
    msleep(500);
    right_degrees(190);
    line_follow(RETURN_LINE_FOLLOW,MOTOR_SPEED_1500);
    left_turn();
    // start of turn up ramp
    drive_till_blk();
    drive(MOTOR_SPEED_1000,MOTOR_SPEED_1000);
    msleep(3350);
    drive(-MOTOR_SPEED_1500,-MOTOR_SPEED_1500);
    msleep(250);
    left_degrees(50);

    /* Go Up the ramp until white */
    go_up_ramp();

    // driving to edge of box
    drive(MOTOR_SPEED_1000, MOTOR_SPEED_1000);
    msleep(1100);
    right_degrees(15);
    // pushing poms into the box
    drive(MOTOR_SPEED_1000, MOTOR_SPEED_1000);
    msleep(500);
    back_till_blk();
    // Drop cones
    left_degrees(25);
    drive(MOTOR_SPEED_1000, MOTOR_SPEED_1000);
    msleep(100);
    ao();
    move_arm_fast_slow_down(ARM_DOWN);
    // opening claw to drop cones
    set_servo_position(CLAW, CLAW_OPEN);
    msleep(500);
    set_servo_position(ARM, ARM_UP);
    msleep(500);
    set_servo_position(CLAW, CLAW_CLOSE-200);
    msleep(100);
    // drive to push cones into edge of upper box
    drive(MOTOR_SPEED_1000, MOTOR_SPEED_1000);
    msleep(1400);
    ao();
    drive(-MOTOR_SPEED_1000, -MOTOR_SPEED_1000);
    msleep(1500);
    back_till_blk();
    left_degrees(160);
    // driving backwards to move to parking spot
    drive(-MOTOR_SPEED_1000,-MOTOR_SPEED_1000);
    msleep(3000);
    ao();
    // right turn to parallel park
    right_degrees(100);  



}


int main() {
    shut_down_in(119);
    enable_servos();
    int start_time = seconds();
    printf("Start Time: %d\n", start_time);
    //msleep(35000);
    move_pick_cone1();
    move_pick_cone2();
    return_cone();
    int end_time = seconds();
    int total_time = end_time - start_time;
    printf("End Time: %d", end_time);
    printf("Total Time: %d",total_time);


}
