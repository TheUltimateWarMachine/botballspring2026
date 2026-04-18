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
int start_seconds;
//Each step of the robot's movement should be in instructions.h!
//Do not clutter this file.
int main() {
    start_seconds = seconds();
    shut_down_in(119);
    enable_servos();
    claw_reset();
    printf("Starting drive to green cube\n");
    drive_to_green_cube_from_starting_box();
    //home_drive_to_green_cube_from_starting_box();
    knock_green_cube(); 
    open_warehouse_doors();
    grab_botguy();
    drive_back_to_starting_area();
    halt_robot("Program ended");
    return EXIT_SUCCESS;
}
