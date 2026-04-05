#include <kipr/wombat.h>
#include <math.h>
#include <stdlib.h>

#define CLAW 1
#define ARM 0
#define MOTORl 0
#define MOTORr 3
#define SENSORl 0
#define SENSORr 3
#define BLACK 3000
#define RIGHT_MOTOR_CORRECTION 1
#define MOTOR_SPEED_500 500
#define MOTOR_SPEED_1000 1000
#define MOTOR_SPEED_1500 1500
#define WHITE 2500

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
        msleep(30);
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


void drive(int speed1,int speed2) {
    mav(MOTORl, speed1);
    mav(MOTORr,speed2* RIGHT_MOTOR_CORRECTION);
    printf("Motor Correction Happening");
}
void drive_till_blk(){
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(1500,1500);
    }
}
void drive_till_white(){
    while (analog(SENSORl) >= WHITE && analog(SENSORr) >= WHITE) {
        drive(1500,1500);
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
    int ticks = -11.11 * degrees;
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
    int ticks = -11.11 * degrees;
    while(gmpc(MOTORl) > ticks && gmpc(MOTORr) > ticks){
        drive(-MOTOR_SPEED_500, MOTOR_SPEED_500);
    }
    ao();
    msleep(50);
}
void back_till_blk(){
    while (analog(SENSORl) <= BLACK && analog(SENSORr) <= BLACK) {
        drive(-1500,-1500);
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
    drive_till_blk();
    line_follow(14000,1500);
    drive_till_white();
    ao();
}
void move_pick_cone1() {
    printf("starting to go to cone1");
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
    printf("Stopped to Pick Cone1");
    move_arm(1848);
    move_claw(1667);
    printf("Closed Claw, Ended Cone1");
}
void move_pick_cone2() {
    enable_servos();
    line_follow(4000,1500);
    ao();
    printf("Stopped to Pick Cone1");
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
    printf("Closed Claw, Ended Cone1");
}

void drop_cone() {
    line_follow(5300,1500);
    left_turn();
    drive_till_blk();

    drive(1000,1000);
    msleep(3700);
    left_60();
    go_up_ramp();
    move_arm(1800);
    move_claw(500);
    right_degrees(50);
}

void return_cone()
{
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


int main() {
    
    move_pick_cone1();
    move_pick_cone2();
    return_cone();

}
