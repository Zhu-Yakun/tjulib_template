#include "vex.h"
#include "tjulib.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
// 底盘电机
motor L1 = motor(PORT1, ratio6_1, true);
motor L2 = motor(PORT2, ratio6_1, false);
motor L3 = motor(PORT3, ratio6_1, true);
motor R1 = motor(PORT4, ratio6_1, false);
motor R2 = motor(PORT5, ratio6_1, true);
motor R3 = motor(PORT6, ratio6_1, false);
// 手柄
controller Controller1 = controller(primary);
// 非底盘电机
motor roller = motor(PORT7, ratio18_1, true);
// imu和编码器
inertial imu = inertial(PORT8);
encoder encoderHorizonal = encoder(Brain.ThreeWirePort.A);
encoder encoderVertical = encoder(Brain.ThreeWirePort.C);
// 气动件
pwm_out gas = pwm_out(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}