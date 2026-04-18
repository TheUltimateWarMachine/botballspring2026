#ifndef LIBRARY_H_INCLUDED
#define LIBRARY_H_INCLUDED 1
#include<kipr/wombat.h>
#include<stdint.h>
#include<stdbool.h>
typedef signed short (*gyro_axis_callback)(void);
void drive(int speed, int dist);
double get_calibrated_gyro_reading(int precision, gyro_axis_callback axis);
void cmpc_all(void);
void gyro_drive_until(gyro_axis_callback gyro_axis, double gyro_bias, int speed, bool (*stop_condition)(int), int condition_arg);
void drive_until(int speed, bool (*stop_condition)(int), int condition_arg);
bool black_detected(int ticks);
void rotate_relative(int speed, double deg);
void halt_robot(char* message);
bool is_tophat_black(int th);
//void one_sensor_line_follow(int sensor, int ticks);
//Claw Servo Ports
#define CLAW_VERT 0
#define CLAW_HORIZ 3
#define lmav(speed) mav(L_MTR, (speed)) //left motor is - because the motor is installed backward
#define rmav(speed) mav(R_MTR, (speed))
#define lgmpc() (-gmpc(L_MTR))
#define rgmpc() (gmpc(R_MTR))
#define gmpc_all() ((lgmpc() + rgmpc()) / 2) //returns the average of both motors' position counters.
#define normalized_gmpc_all() ( (abs(lgmpc()) + abs(rgmpc()) ) / 2)
//wait a short time for mechanical reasons or to slow down infinite loop execution.
#define sgn(val) (((val) > 0) - ((val) < 0)) //sign of x (-1, 0, 1)
#define UNREFERENCED(variable) (void)variable //Use this for variables/parameters that go unreferenced.
//Recommended when letting motors stop to include a mechanical wait.
#define mechanical_wait() do { msleep(150); } while(false);
#define short_mechanical_wait() do { msleep(50); } while(false); //wait a very short time for mechanical reasons or to slow down infinite loop execution.
#define SYNC_GAIN    0.2f // Steering correction intensity, for uneven motors/wheels

// get time in milliseconds
long msecs() {
    // seconds() is float, convert to long
    return (long)(seconds() * 1000);
}

//Clears both of the motors' position counters
void cmpc_all(void) {
    cmpc(L_MTR);
    cmpc(R_MTR);
}

/*rotates the robot an angle relative to its current one. cmpc's.
* speed = motor speed while rotating. Speed will be absolute valued so don't bother putting in -speed values.
* deg = #degrees to turn: +values = clockwise, -values = counterclockwise
*/
void rotate_relative(int speed, double deg) {
    double ticks_per_deg = 11.8;
    //If the motors have too much momentum built up, it will overshoot.
    double degree_corrector = 10; 
    double min_degrees_corrected = 30; //When have the motors built up enough momentum to cause overshoot issues?
    if(abs(deg) > min_degrees_corrected)
        deg += -sgn(deg) * degree_corrector;
    cmpc_all();
    speed = abs(speed);
    while(normalized_gmpc_all() < abs(deg * ticks_per_deg)) {
        lmav(speed * sgn(deg));
        rmav(speed * -sgn(deg));
        short_mechanical_wait();
    }
    ao();
    mechanical_wait();
}

#define MOTOR_TURN_START_RATE 15.0 // Higher is more aggressive
#define MOTOR_TURN_STOP_RATE 8.0 // Motors have momentum, need more time to stop
// Velocity range for mav() is -1500 to 1500. 800-1000 is the "sweet spot" for accuracy.
#define MOTOR_TURN_MAX_SPEED 800
#define MOTOR_TURN_MIN_SPEED 200
#define MOTOR_TURN_SLOW_START_RAMP_TICKS 400
#define MOTOR_TURN_SLOW_STOP_RAMP_TICKS 600
#define MOTOR_MOV_MAX_SPEED 1200
#define MOTOR_MOV_MIN_SPEED 400 // For square up

void right_sensor_line_follow(int ticks) {
    const int calibration_base = 300;
    const int calibration_correct = 30;
    const int travel_base = 800;
    const int travel_correction = 45;
    const int too_angled_threshold = 1000;
    //look for black line
    while(!is_tophat_black(R_TH)) {
        lmav(calibration_base + calibration_correct);
        rmav(calibration_base - calibration_correct);
        short_mechanical_wait();
    }
    //if we spent too long calibrating we are too angled, we must correct quickly
    if(normalized_gmpc_all() > too_angled_threshold) {
        lmav(-calibration_base);
        rmav(calibration_base);
        msleep(180);
    }
    //Ticks only are counted post-calibration
    cmpc_all();
    while(normalized_gmpc_all() < abs(ticks)) {
		if(is_tophat_black(R_TH)) {
            lmav(travel_base - travel_correction);
            rmav(travel_base + travel_correction);
        } else {
            lmav(travel_base + travel_correction);
            rmav(travel_base - travel_correction);
        }
        short_mechanical_wait();
    }
}

void rotate_smooth(double deg) {
    const double ticks_per_deg = 11.99;
    // Because we slow down as we approach the end, momentum is not
    // a problem, no degree correction needed
    cmpc_all();
    int total_correction = 0;
    int remaining = abs(deg * ticks_per_deg) - normalized_gmpc_all();
    while(remaining > 0) {
        int auto_speed = remaining * MOTOR_TURN_START_RATE;
        // slow start
        if (normalized_gmpc_all() < MOTOR_TURN_SLOW_START_RAMP_TICKS) {
            auto_speed =  normalized_gmpc_all() * MOTOR_TURN_START_RATE;
        }
        int left_ticks = abs(get_motor_position_counter(L_MTR));
        int right_ticks = abs(get_motor_position_counter(R_MTR));
        int error = left_ticks - right_ticks;
        int correction = (int)(error * SYNC_GAIN);
        total_correction += abs(correction);
        // slow stop
        if (remaining < MOTOR_TURN_SLOW_STOP_RAMP_TICKS) {
            auto_speed = remaining * MOTOR_TURN_STOP_RATE;
        }
        // Clamp the speed between our safe limits
        if (auto_speed > MOTOR_TURN_MAX_SPEED) auto_speed = MOTOR_TURN_MAX_SPEED;
        if (auto_speed < MOTOR_TURN_MIN_SPEED) auto_speed = MOTOR_TURN_MIN_SPEED;
        //lmav((auto_speed - correction) * sgn(deg));
        //rmav((auto_speed + correction) * -sgn(deg));
        lmav((auto_speed) * sgn(deg));
        rmav((auto_speed) * -sgn(deg));
        msleep(10);
        remaining = abs(deg * ticks_per_deg) - normalized_gmpc_all();
        //printf("rotate_smooth: remaining %d speed %d\n", remaining, auto_speed);
    }
    freeze(L_MTR);
    freeze(R_MTR);
    printf("rotate_smooth: tick_guess %lf, left_ticks %d, right_ticks %d, total_correction %d\n",
           deg * ticks_per_deg, gmpc(L_MTR), gmpc(R_MTR), total_correction);
}

//Returns if tophat at port [th] is reading black.
bool is_tophat_black(int th) {
    return analog(th) > BLACK_THRESHOLD;
}

//drive_until compatible version of is_tophat_black, though this one checks both sensors. arg is ignored
bool ccblack_detected(int supplied_arg) {
    UNREFERENCED(supplied_arg);
    return is_tophat_black(L_TH) || is_tophat_black(R_TH);
}

//opposite of ccblack_detected();
bool ccwhite_detected(int supplied_arg) {
    UNREFERENCED(supplied_arg);
    return !ccblack_detected(0);
}

//drive_until-compatible conditional. Returns if the amount of ticks driven > the ticks supplied into condition_arg.
bool ccticks_reached(int supplied_ticks) {
    return supplied_ticks > gmpc_all();
}

/*Calibrates the gyroscope to get a starting point for future functions.
* precision = how precise should the reading be? Do not make too big, as each increment incurs a mechanical wait.
* axis = gyro_x, gyro_y, or gyro_z. Which axis should be calibrated?
*/
double get_calibrated_gyro_reading(int precision, gyro_axis_callback axis) {
    double bias = 0;
    int i = 0;
    while(i < precision) {
        bias += axis();
        i++;
        mechanical_wait();
    }
    return bias / (double)precision;
}



/* Simply drives forward(+speed)/backward(-speed). Resets motor position counters beforehand.
* speed = motor speed while driving
* dist = distance to drive (in motor ticks)
*/
void drive(int speed, int dist) {
    cmpc_all();
    while(normalized_gmpc_all() < dist) {
        lmav(speed);
        rmav(speed);
        short_mechanical_wait();
    }
    ao();
    mechanical_wait();
}

/*Same as drive() but allows for custom conditionals.
* speed = motor speed while driving
* stop_condition = conditional function to be ran. If true is returned, the driving will stop. Takes one argument, into which this function supplies [condition_arg].
*/
void drive_until(int speed, bool (*stop_condition)(int), int condition_arg) {
    cmpc_all();
    while(!stop_condition(condition_arg)) {
        lmav(speed);
        rmav(speed);
        short_mechanical_wait();
    }
    ao();
    mechanical_wait();
}

/* Drive with the gyro until [stop_condition] is met. cmpc's at the beginning
* gyro_axis = gyro axis to be used
* gyro_bias = Last gyro calibration value to be used as a baseline
* speed = motor speed while driving. Subject to variations as the motors' speed is varied in line with gyro measurement
* stop_condition = conditional function to be ran. If true is returned, the driving will stop. Takes one argument, into which this function supplies [condition_arg].
*/
void gyro_drive_until(gyro_axis_callback gyro_axis, double gyro_bias, int speed, bool (*stop_condition)(int), int condition_arg) {
    int deviation_corrector = 5;
    cmpc_all();
    double delta = 0, dev = 0;
    while(!stop_condition(condition_arg)) {
        delta = dev / deviation_corrector;
        lmav(speed + delta);
        rmav(speed + delta);
        short_mechanical_wait();
        dev = gyro_axis() - gyro_bias;
    }
    ao();
    mechanical_wait();
}

/*squares up against a black line. cmpc_all's.
* speed = motor speed while moving; will be changed a lot as the square up completes. Absolute value is taken; don't submit negative speeds
* tick_guess = supplied guess on how many ticks away the black line is
*/
void straight_square_up(int speed, int tick_guess) {
    cmpc_all();
    speed = abs(speed);
    int dynspeed = speed; //dynamic speed is adjusted to make the square-up more accurate
    //If black is detected on both tophats, we have successfully squared up
    bool tape_detected = false;
    while(!(is_tophat_black(L_TH) && is_tophat_black(R_TH))) {
        //printf("straight_square_up: left black %d right black %d\n", is_tophat_black(L_TH), is_tophat_black(R_TH));
        //If we surpass the tick_guess and haven't hit the black line, slow down; we are very close.
        if(!tape_detected && tick_guess < normalized_gmpc_all()) {
            dynspeed = speed / 3;
        }
        if(is_tophat_black(L_TH) || is_tophat_black(R_TH)) {
            dynspeed = speed / 5;
            tape_detected = true;
        }
        if(is_tophat_black(L_TH) && !is_tophat_black(R_TH)) {
            lmav(-dynspeed);
            rmav(+dynspeed);
            tape_detected = true;
        } else if(!is_tophat_black(L_TH) && is_tophat_black(R_TH)) {
            lmav(+dynspeed);
            rmav(-dynspeed);
            tape_detected = true;
        } else { //L_TH white && R_TH white
            lmav(+dynspeed);
            rmav(+dynspeed);
        }
        msleep(10);
    }
    ao();
    mechanical_wait();
    printf("straight_square_up: tick_guess %d, left_ticks %d, right_ticks %d\n",
           tick_guess, gmpc(L_MTR), gmpc(R_MTR));
}

// --- PORT ASSIGNMENTS ---
#define PORT_HORZ CLAW_HORIZ
#define PORT_VERT CLAW_VERT

// --- PHYSICAL CALIBRATION ---
// KIPR documentation says servo takes 110 msecs for 60 degrees
// For 180 degrees, 330 msecs. There are 2047 units so, time
// per unit is 330 /2047 = 0.16. Use 0.20 to account for weight
// and safety.
#define SERVO_MS_PER_UNIT_LIMIT 0.20f  
#define SERVO_MIN_SYSTEM_DELAY  10     
#define SERVO_MIN_STEP_SIZE     5       

// --- VERTICAL GRAVITY SETTINGS --- 
// NOTE: On this robot, smaller unit values = higher physical elevation.
#define SERVO_VERT_UP_SCALE     1.5f  // Fighting gravity: 1.5x slower (Safe)
// While going down, internal gears of the servo
// have a maximum physical rotation speed (the "slew rate")
// So, they are not going to go faster even though gravity is
// pulling them.
#define SERVO_VERT_DOWN_SCALE   1.0f  // Aided by gravity: Physical Max Speed

// --- WEIGHT DEFINITIONS (In Grams) ---
#define WEIGHT_EMPTY   0
#define WEIGHT_CUBE    30
#define WEIGHT_BOTGUY  90

// --- PHYSICS DERIVATION CONSTANTS ---
// Snap ratio is the pct of remaining units to move in a step
// Higher ratio means move faster initially.
// We slow down if loaded, and slow down if going vertical
// Note: Servos manage gravity pull while going down just fine
// due to internal friction. It is pulling weight up which needs help.
#define RATIO_BASE     0.25f   // Snap ratio for 0g load
#define RATIO_PENALTY  0.002f  // Subtract 0.002 from ratio per gram
#define LOAD_TIME_MOD  0.01f   // Add 1% delay per gram when lifting

// --- GLOBAL STATE --- Current weight in the claw
int CURRENT_WEIGHT_G = WEIGHT_EMPTY; 
extern int start_seconds;

void halt_robot(char* message) {
    int time_taken = seconds() - start_seconds;
    printf("CRITICAL HALT: %s, took %d seconds\n", message, time_taken);
    ao(); disable_servos(); exit(1);
}

// --- THE ENGINE (Auto-Deriving Ratio and Scale from Grams) ---
void _move_servo_engine(int port, int target_pos, float up_scale, float down_scale) {
    if (target_pos < 0 || target_pos > 2047) {
      printf("port %d, target_pos %d\n", port, target_pos);
      halt_robot("Invalid Position");
    }

    // 1. DERIVE RATIO: Applied to ALL servos (heavier = gentler stop)
    float derived_ratio = RATIO_BASE - (CURRENT_WEIGHT_G * RATIO_PENALTY);
    if (derived_ratio < 0.08f) { derived_ratio = 0.08f; }

    enable_servo(port);
    int current_pos = get_servo_position(port);
    int num_steps = 0;
    long start_ms = msecs();

    //  Proportional Loop: Moves in decreasing increments
    printf("%s, %d: Starting Servo %d, cur pos %d, desired post %d\n", __FUNCTION__, __LINE__,
           port, current_pos, target_pos);
    while (abs(target_pos - current_pos) > SERVO_MIN_STEP_SIZE) {
        int remaining = target_pos - current_pos;
        int step = (int)(remaining * derived_ratio);

        if (abs(step) < SERVO_MIN_STEP_SIZE) {
            step = (remaining > 0) ? SERVO_MIN_STEP_SIZE : -SERVO_MIN_STEP_SIZE;
        }

        // Logic: Is this a vertical move?
        float active_scale = (target_pos < current_pos) ? up_scale : down_scale;

        // 2. PORT-SPECIFIC LOAD: Only apply gravity penalty if moving PORT_VERT UP
        if (port == PORT_VERT && target_pos < current_pos) {
            active_scale += (float)(CURRENT_WEIGHT_G * LOAD_TIME_MOD);
        }

        // Compute time to wait. Slow down for heavier weights (and lifting up)
        int final_delay = (int)(abs(step) * SERVO_MS_PER_UNIT_LIMIT * active_scale);
        if (final_delay < SERVO_MIN_SYSTEM_DELAY) { final_delay = SERVO_MIN_SYSTEM_DELAY; }

        current_pos += step;
        set_servo_position(port, current_pos);
        msleep(final_delay);
        //printf("move_servo_engine: Working Servo %d, cur pos %d, step %d, delay %d\n", port, current_pos, step, final_delay);
        num_steps++;
    }
    // 3. FINAL SNAP: Reaches the exact target coordinate before power-down.
    // This clears the 'Minimum Step' gap and ensures precision.
    set_servo_position(port, target_pos);
    msleep(50);
    // Protects motor from stalling if obstructed and saves battery
    // It would still hold the current position, by internal and
    // external friction
    disable_servo(port);
    printf("%s, %d: Finished Servo %d, num_steps %d, time taken %ld msecs\n", __FUNCTION__, __LINE__,
           port, num_steps, msecs() - start_ms);
}

// --- WRAPPERS ---
void move_vert_servo(int target_pos) {
    _move_servo_engine(PORT_VERT, target_pos, SERVO_VERT_UP_SCALE, SERVO_VERT_DOWN_SCALE);
}

void move_horz_servo(int target_pos) {
    _move_servo_engine(PORT_HORZ, target_pos, 1.0f, 1.0f);
}

#define LEFT_MOTOR L_MTR
#define RIGHT_MOTOR R_MTR

// --- POLARITY FIX ---
// If a motor is plugged in backwards, its ticks count negative when moving forward.
// Set to 1 for normal orientation, -1 for reversed.
#define LEFT_DIR   1  
#define RIGHT_DIR 1 

// --- CALIBRATION ---
#define ACCEL_STEP   80   // Velocity increase per 20ms loop
#define RAMP_DIST    400  // Standard ramp distance for long moves

void drive_smooth(int target_ticks) {
    clear_motor_position_counter(LEFT_MOTOR);
    clear_motor_position_counter(RIGHT_MOTOR);

    int current_vel = 0;
    int direction = target_ticks > 0 ? 1 : -1;
    int total_correction = 0;
    
    // DYNAMIC RAMP CALCULATION:
    // If the move is shorter than 2x RAMP_DIST, we shrink the ramp to 
    // exactly 50% of the distance to create a symmetrical triangle profile.
    int effective_ramp = (int)fmin(RAMP_DIST, abs(target_ticks) / 2);

    while (1) {
        // Use abs() to ensure we are comparing positive distance traveled,
        // regardless of the physical motor orientation.
        int left_ticks = abs(get_motor_position_counter(LEFT_MOTOR));
        int right_ticks = abs(get_motor_position_counter(RIGHT_MOTOR));
        
        // Loop exit condition
        //printf("drive_smooth: left %d right %d target %d\n", left_ticks, right_ticks, target_ticks);
        if (left_ticks >= abs(target_ticks)) break;
        int remaining = target_ticks - left_ticks;

        // --- 1. THE BRAKE CHECK (High Priority) ---
        // We check the remaining distance first to prioritize slowing down.
        // This prevents momentum from causing an overshoot.
        if (remaining < effective_ramp) {
            current_vel = (remaining * MOTOR_MOV_MAX_SPEED) / effective_ramp;
            
            // KINETIC FRICTION FLOOR: Prevents the robot from stalling short
            // of the target due to mechanical resistance at low speeds.
            if (current_vel < 400) current_vel = 400;
        } 
        // --- 2. THE START-UP CHECK ---
        else if (left_ticks < effective_ramp) {
            // STATIC FRICTION MANAGEMENT: Ramping from 0 allows tires to grip
            // the board gently, preventing wheel-spin and encoder inaccuracy.
            current_vel += ACCEL_STEP;
            if (current_vel > MOTOR_MOV_MAX_SPEED) current_vel = MOTOR_MOV_MAX_SPEED;
        } 
        else {
            current_vel = MOTOR_MOV_MAX_SPEED; // Cruise phase
        }

        // --- 3. ACTIVE SYNC CORRECTION ---
        // We compare wheel encoders in real-time. If one wheel is ahead, 
        // we shift target velocities to pull the robot back into a straight line.
        int error = left_ticks - right_ticks;
        int correction = (int)(error * SYNC_GAIN);
        total_correction += abs(correction);

        // --- 4. FINAL MOTOR COMMAND ---
        // We multiply the calculated velocity by the DIR constant.
        // This ensures the reversed motor receives the negative signal it needs.
        mav(LEFT_MOTOR, (current_vel - correction) * LEFT_DIR * direction);
        mav(RIGHT_MOTOR, (current_vel + correction) * RIGHT_DIR * direction);
        msleep(10); 
    }

    // Freeze both motors for a precise, electronically locked stop
    // use ao() for a rolling stop
    freeze(LEFT_MOTOR);
    freeze(RIGHT_MOTOR);
    printf("drive_smooth: total_correction %d, target_ticks %d, left_ticks %d, right_ticks %d\n",
           total_correction, target_ticks, get_motor_position_counter(LEFT_MOTOR), get_motor_position_counter(RIGHT_MOTOR));
}

void square_up_smooth(int tick_guess) {
    cmpc_all();
    int speed = MOTOR_MOV_MAX_SPEED;
    speed = abs(speed);
    const int direction = sgn(tick_guess);
    int dynspeed = speed; //dynamic speed is adjusted to make the square-up more accurate
    //If black is detected on both tophats, we have successfully squared up
    bool tape_detected = false;
    while(!(is_tophat_black(L_TH) && is_tophat_black(R_TH))) {
        int cur_ticks = normalized_gmpc_all();
        // Slow start
        if (abs(cur_ticks) < 400) {
            // 200 min speed, 1200 max speed
            speed = fmax(cur_ticks * 10, 100);
            speed = fmin(speed, 1200);
        }
        dynspeed = speed;

        //If we surpass the tick_guess and haven't hit the black line, slow down; we are very close.
        if(!tape_detected && tick_guess < cur_ticks) {
            dynspeed = speed / 3;
        }
        if(is_tophat_black(L_TH) || is_tophat_black(R_TH)) {
            dynspeed = speed / 5;
            tape_detected = true;
        }
        dynspeed *= direction;
        if(is_tophat_black(L_TH) && !is_tophat_black(R_TH)) {
            lmav(-dynspeed);
            rmav(+dynspeed);
            tape_detected = true;
        } else if(!is_tophat_black(L_TH) && is_tophat_black(R_TH)) {
            lmav(+dynspeed);
            rmav(-dynspeed);
            tape_detected = true;
        } else { //L_TH white && R_TH white
            lmav(+dynspeed);
            rmav(+dynspeed);
        }
        //printf("%s, %d: cur_ticks %d, dynspeed %d, left black %d right black %d\n", __FUNCTION__, __LINE__,
              //cur_ticks, dynspeed, is_tophat_black(L_TH), is_tophat_black(R_TH));
        msleep(10);
    }
    // Freeze both motors for a precise, electronically locked stop
    // use ao() for a rolling stop
    freeze(LEFT_MOTOR);
    freeze(RIGHT_MOTOR);
    //printf("%s, %d: DONE tick_guess %d, left_ticks %d, right_ticks %d\n", __FUNCTION__, __LINE__,
      //     tick_guess, gmpc(L_MTR), gmpc(R_MTR));
}

// --- GYRO CALIBRATION ---
// Adjust this factor based on testing (e.g., if you turn 90 but it says 100, lower this)
#define GYRO_SCALE 1.0 // 0.0012f 


// smooth gyro_z reading
// average over a few samples
int get_gyro_z_smooth() {
    int sum = 0;
    int n = 10;
    for (int i = 0; i < n; i++) sum += gyro_z();
    return sum / n;
}

void gyro_turn(int degrees) {
    // 1. Calibrate Bias (Robot must be STILL)
    double bias = 0;
    float num_bias_samples = 20;
    for(int i=0; i< num_bias_samples; i++) {
        bias += get_gyro_z_smooth();
        msleep(1);
    }
    bias /= num_bias_samples;
    printf("Gyro Z Bias is %f\n", bias);

    double current_angle = 0;
    int left_sign = (degrees > 0) ? 1 : -1;
    int right_sign = (degrees > 0) ? -1 : 1;

    // 2. Turn Loop
    while (abs(current_angle) < abs(degrees)) {
        // Integrate: add current speed to the total angle
        // Subtract bias to prevent "fake" turning while moving
        current_angle += (get_gyro_z_smooth() - bias) * GYRO_SCALE;

        // Simple Proportional Slowdown: get slower as we reach the target
        int remaining = abs(degrees) - abs(current_angle);
        int auto_speed = remaining * MOTOR_TURN_START_RATE;
        // Clamp the speed between our safe limits
        if (auto_speed > MOTOR_TURN_MAX_SPEED) auto_speed = MOTOR_TURN_MAX_SPEED;
        if (auto_speed < MOTOR_TURN_MIN_SPEED) auto_speed = MOTOR_TURN_MIN_SPEED;

        mav(LEFT_MOTOR, auto_speed * left_sign * LEFT_DIR);
        mav(RIGHT_MOTOR, auto_speed * right_sign * RIGHT_DIR);

        msleep(10); // High frequency for accurate integration
        printf("gyro_turn cur is %d remaining %d speed %d\n", get_gyro_z_smooth(), remaining, auto_speed);
    }

    freeze(LEFT_MOTOR);
    freeze(RIGHT_MOTOR);
}

#endif //defined(LIBRARY_H_INCLUDED)
