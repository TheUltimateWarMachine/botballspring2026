#include<kipr/wombat.h>
#include<stdlib.h>
#include<math.h>
#include<stdint.h>
#include<stdbool.h>
#define VERTICAL_SERVO 3
#define L_MTR 3
#define R_MTR 1
#define L_TH 1
#define R_TH 0
#define BLACK_THRESHOLD 1200 //any tophat reading past this is black.
#include"library.h"
#include"instructions.h"
//Each step of the robot's movement should be in instructions.h!
//Do not clutter this file.
int main() {
    //square_up_smooth(1300);
    //drive_smooth(2600);
    // straight_square_up(1500, 2000);
    rotate_smooth(90); // Turn right towards middle of board
    halt_robot("Testing ended");
    
    shut_down_in(119);
    enable_servos();
    claw_reset();
    drive_to_green_cube_from_starting_box();
    knock_green_cube(); 
    open_warehouse_doors();
    grab_botguy();
    halt_robot("Program ended");
    return EXIT_SUCCESS;
}
