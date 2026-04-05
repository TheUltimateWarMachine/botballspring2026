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
    drive_smooth(2600);
    rotate_smooth(90); // Turn right towards middle of board
    printf("Driving to middle line\n");
    drive(1500, 5600);
    square_up_smooth(300);
    drive(800, 850); // Go beyond the middle line
    rotate_relative(1500, -90); // Turn towards the doors
    drive_smooth(-10);
    straight_square_up(800, 100);
    mechanical_wait();
    printf("Done drive to green cube\n");
}

void knock_green_cube(void) {
    printf("starting knock green cube\n");
    const int claw_horiz_open = 600;
    const int claw_horiz_close = 1650;
    const int door_handle_height = 1600; //Height in servo position of the door handle from our stopping location
    move_horz_servo(claw_horiz_open);
    move_vert_servo(1970); // Claw down to grab cube
    // drive_smooth(distance)
    drive_smooth(605);
    mechanical_wait();
    move_horz_servo(claw_horiz_close); //close the claw to grab on the cube
    msleep(850);
    move_vert_servo(1525); // lift claw up
    mechanical_wait();
    drive(-1000, 220); //drive back to avoid the cubee rubbing against thge door
    rotate_smooth(-60); // Turn left
    msleep(1000);
    move_vert_servo(1910); // claw down to drop the cube
    mechanical_wait();
    printf("About to drop the cube, claw_horz_open is %d\n", claw_horiz_open);
    move_horz_servo(claw_horiz_open); // open claw to drop cube
    rotate_smooth(-3); // Rotate to disengage left (stationary) claw arm
    move_vert_servo(door_handle_height);
    rotate_smooth(60);
    printf("Completed knock green cube\n");
} 

void open_warehouse_doors(void) {
    const int door_open_dist = 1500; //How far we can push the door with the claw (In servo positions)
	move_horz_servo(door_open_dist);
    msleep(800);
    drive_smooth(1000); //drive forward so the claw can reach the doors
    short_mechanical_wait();
    rotate_smooth(-60); //this one opens the left door
    msleep(900);
    rotate_smooth(45);
    drive(250, 180); //drive forward so that the claw engages the right door
    msleep(900);
    rotate_smooth(45);
    move_horz_servo(800); // both of them combine to open the right door
    msleep(800);
    mechanical_wait();
    msleep(800);    
}

void grab_botguy(void) {
    // rotate_smooth(degree)
    rotate_smooth(-70); // turn towards botguy after opening door
    const int botguy_grab_height = 1850;
    move_horz_servo(1100);
    move_vert_servo(botguy_grab_height);
    msleep(800);
    drive_smooth(900); // Move forward
    move_horz_servo(1700); // close claw to grab botguy 
    move_vert_servo(botguy_grab_height - 150); // lift botguy
    drive_smooth(-1200); // drive back
}
#endif //defined(INSTRUCTIONS_H_INCLUDED)
