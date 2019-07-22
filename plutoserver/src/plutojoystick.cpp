#include <plutoserver/plutojoystick.h>
#include <iostream>


using namespace std;

ros::Publisher command_pub;
ros::Subscriber joy_sub;

plutodrone::PlutoMsg cmd;

int trim_roll = 0;
int trim_pitch = 0;

int axesValue;

float mapping(float x, float inMin, float inMax, float outMin, float outMax) {
    x = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

    if (x < outMin)
      return outMin;
    else if (x > outMax)
      return outMax;
    else
      return x;
}

void arm(){
  cmd.isAutoPilotOn = 0;
  cmd.rcRoll=1500;
  cmd.rcYaw=1500;
  cmd.rcPitch =1500;
  cmd.rcThrottle =1000;
  cmd.rcAUX4 =1500;
  command_pub.publish(cmd);
  usleep(200000);
}

void disarm(){
  cmd.rcThrottle =1300;
  cmd.rcAUX4 = 1200;
  command_pub.publish(cmd);
  usleep(200000);
}

void box_arm(){
  cmd.isAutoPilotOn = 0;
  cmd.rcRoll=1500;
  cmd.rcYaw=1500;
  cmd.rcPitch =1500;
  cmd.rcThrottle =1500;
  cmd.rcAUX4 =1500;
  command_pub.publish(cmd);
}

void take_off(){
  disarm();
  box_arm();
  cmd.commandType = 1;
  command_pub.publish(cmd);
}

void land() {
  cmd.commandType = 2;
  command_pub.publish(cmd);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[ARM_BTN]) {
      if(cmd.rcAUX4 != 1500)
        arm();
      else
        disarm();
    }

    else if(joy->buttons[TAKE_OFF_BTN]) {
        take_off();
    }

    else if(joy->buttons[LAND_BTN]) {
        land();
    }

    else if(abs(axesValue = -joy->axes[TRIM_ROLL_AXES])) {
        trim_roll += ( 3 * (axesValue));
    }

    else if(abs(axesValue = joy->axes[TRIM_PITCH_AXES])) {
        trim_pitch += ( 3 * (axesValue));
    }

    else if(joy->buttons[TRIM_SAVE_BTN]) {
        cmd.trim_roll = trim_roll;
        cmd.trim_pitch = trim_pitch;
        trim_roll = 0;
        trim_pitch = 0;
        command_pub.publish(cmd);
        usleep(200000);
    }

    else if(joy->buttons[AUTO_PILOT_BTN]) {
       if(cmd.isAutoPilotOn == 1)
         cmd.isAutoPilotOn = 0;
        else
          cmd.isAutoPilotOn = 1;

         cout<<"isAutoPilotOn: "<< cmd.isAutoPilotOn <<endl;

    }

    cmd.rcRoll = mapping(-joy->axes[RC_ROLL_AXES], -1, 1, 1000, 2000);
    cmd.rcPitch = mapping(joy->axes[RC_PITCH_AXES], -1, 1, 1000, 2000);
    cmd.rcYaw = mapping(-joy->axes[RC_YAW_AXES], -1, 1, 1000, 2000);
    cmd.rcThrottle = mapping(joy->axes[RC_THROTTLE_AXES], -1, 1, 1000, 2000);
    cmd.commandType = 0;
    cmd.trim_roll = 0;
    cmd.trim_pitch = 0;
    command_pub.publish(cmd);

}

int main(int argc, char** argv)
{
     ros::init(argc, argv, "plutojoystick");
     ros::NodeHandle n;

     command_pub = n.advertise<plutodrone::PlutoMsg>("drone_command", 1000);
     joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

     cmd.rcRoll =1500;
 		 cmd.rcPitch = 1500;
 		 cmd.rcYaw =1500;
 	   cmd.rcThrottle =1500;
 		 cmd.rcAUX1 =1500;
 		 cmd.rcAUX2 =1500;
 		 cmd.rcAUX3 =1500;
 		 cmd.rcAUX4 =1000;
 		 cmd.commandType = 0;
     cmd.trim_roll = 0;
     cmd.trim_pitch = 0;
     cmd.isAutoPilotOn = 0;

     ros::spin();

     return 0;
}
