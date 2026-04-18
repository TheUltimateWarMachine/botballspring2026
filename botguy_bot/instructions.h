#ifndef INSTRUCTIONS_H_INCLUDED
#define INSTRUCTIONS_H_INCLUDED 1

//Reset the claw positions
void claw_reset(void) {
	move_vert_servo(1600);
    move_horz_servo(1600); // Don't fully open to avoid hitting the raised platform while driving to the black line
}

void drive_to_green_cube_from_starting_box(void) {
    printf("starting drive to green cube\n");
    square_up_smooth(1300);
    drive_smooth(2640);
    rotate_smooth(92.20); // Turn right towards middle of board
    printf("Driving to middle line\n");
    //drive_smooth(5600);
    square_up_smooth(6400);
    drive_smooth(770); // Go beyond the middle line
    rotate_relative(1500, -90); // Turn towards the doors
    drive_smooth(-10);
    square_up_smooth(100);
    printf("Done drive to green cube\n");
}

// Version of above code at home pingpong table
void home_drive_to_green_cube_from_starting_box(void) {
   	square_up_smooth(1300);
    drive_smooth(-400);
    rotate_smooth(90); // Turn right towards middle of board
    //square_up_smooth(3200);
    rotate_smooth(-90);
    square_up_smooth(200);
    msleep(2000);
    drive_smooth(650);
}

void knock_green_cube(void) {
    printf("starting knock green cube\n");
    const int claw_horiz_open = 600;
    const int claw_horiz_close = 1650;
    const int claw_vert_grab = 1970; // This low to grab cube
    const int door_handle_height = 1600; //Height in servo position of the door handle from our stopping location
    move_horz_servo(claw_horiz_open);
    move_vert_servo(claw_vert_grab); // Claw down to grab cube
    drive_smooth(605);
    mechanical_wait();
    move_horz_servo(claw_horiz_close); //close the claw to grab on the cube
    move_vert_servo(1525); // lift claw up
    mechanical_wait();
    drive_smooth(-220); //drive back to avoid the cubee rubbing against thge door
    rotate_smooth(-60); // Turn left
    drive_smooth(800); //go away from highh trafic area to avoid robots stepping on it
    move_vert_servo(claw_vert_grab); // claw down to drop the cube
    printf("About to drop the cube, claw_horz_open is %d\n", claw_horiz_open);
    // Before we drop the cube, realize that the foam of the cube is compressed
    // by the arms. The left arm is immovable, so if we quickly open the right
    // arm, the compressed part of the foam against the fixed arm is going to
    // recoil and the cube would jump away from the recoil.
    // So loosen the claw a bit, reducing the compression, BEFORE opening it completly
    move_horz_servo(claw_horiz_close - 85); // loosen the compression
    move_horz_servo(claw_horiz_open); // open claw to drop cube
    rotate_smooth(-3); // Rotate to disengage left (stationary) claw arm
    move_vert_servo(door_handle_height);
    drive_smooth(-800); //COME BACK AFTER DROPPING
    rotate_smooth(62);
    printf("Completed knock green cube\n");
} 

void open_warehouse_doors(void) {
    const int door_open_dist = 1300; //How far we can push the door with the claw (In servo positions)
	move_horz_servo(door_open_dist);
    drive_smooth(970); //drive forward so the claw can reach the doors
    short_mechanical_wait();
    rotate_smooth(-50); //this one opens the left door
    move_horz_servo(1700);
    rotate_smooth(50); // aim in between the doors
    drive_smooth(240); //drive forward so that the claw engages the right door
    move_horz_servo(950); // both of them combine to open the right door
    move_horz_servo(1600); // close the claw a bit so that it doesn't hit the door on the way back  
}

void grab_botguy(void) {
    const int botguy_grab_height = 1810;
    const int botguy_grab_claw_close_pos = 1900;
    move_vert_servo(botguy_grab_height);
    drive_smooth(500); // Move forward
    move_horz_servo(1100); // open claw
    drive_smooth(500);
    move_horz_servo(botguy_grab_claw_close_pos - 30);
    move_horz_servo(botguy_grab_claw_close_pos); // close claw to grab botguy 
    CURRENT_WEIGHT_G = WEIGHT_BOTGUY; // slow down vert servo movements because claw is loaded
    move_vert_servo(botguy_grab_height - 200); // lift botguy, below the top bar, but above the pipes
    rotate_smooth(10); // turn to avoid hitting the left door
    // Lift in small increments
    move_vert_servo(botguy_grab_height - 300);
}

void drive_back_to_starting_area(void) {
    drive_smooth(-1700); //clear the door
    rotate_smooth(-10); // Fix angle back to avoid driving backward at a sharp angle
    square_up_smooth(-700); // drive back to tape
    drive_smooth(-815); // Get further away from green cube
    move_vert_servo(1200); // Lift up to avoid hitting the green cube
    rotate_smooth(-90); // turn back towards starting square
    right_sensor_line_follow(5900);
    rotate_smooth(-72);
    drive_smooth(2500);
    move_vert_servo(1700);
    move_horz_servo(800);
}
#endif //defined(INSTRUCTIONS_H_INCLUDED)
