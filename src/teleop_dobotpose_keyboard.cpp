#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "dobot/DobotPoseMsg.h"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "teleop_dobotpose_keyboard");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<dobot::DobotPoseMsg>("dobot/set_dobot_pose", 10);
    ros::Rate loop_rate(10);
    float input_x = 0.0, input_y = 0.0, input_z = 0.0, input_r = 0.0;

    cout << endl;
    cout << "Move: Input the relative pose coordinate to move dobot: x y z r" << endl;
    cout << "Zero: Input a number in whatever x or y or z or r with a over 1000 number" << endl;
    cout << "Quit: Input any letter to quit" << endl;
    cout << " ----- " << endl;

    while (ros::ok()) {
        cout << ": ";
        dobot::DobotPoseMsg sendPose;
        if (!(cin >> input_x >> input_y >> input_z >> input_r)) {
            cin.clear();
            cin.sync();
            cout << "INFO: Quit from teleop_dobotpose_keyboard" << endl;
            return -1;
        }
        sendPose.control = 0;
        if (input_x > 1000 || input_y > 1000 || input_z > 1000 || input_r > 1000) {
            sendPose.control = 1;
            cout << "INFO: Dobot set to zero position" << endl;
        }
        else {
            cout << "INFO: Dobot will move to relative coordinate: ( " << input_x << ", " << input_y << ", " << input_z << ", " << input_r << " )" << endl;
        }
        sendPose.mode = 0;      // send position coordinate is relative
        sendPose.x = input_x;
        sendPose.y = input_y;
        sendPose.z = input_z;
        sendPose.r = input_r;
        pub.publish(sendPose);
        cout << " ----- " << endl;
        ros::spinOnce();

    }
    return 0;
}
