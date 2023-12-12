using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern controller Controller1;
extern motor roller;  // 非底盘电机roller

extern inertial imu;
extern encoder encoderHorizonal;
extern encoder encoderVertical;

extern pwm_out gas;   // 气动件gas

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );