#include <iostream>
#include "ros/ros.h"
#include "dobot/DobotPose.h"

using namespace std;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "teleop_dobotpose_keyboard");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<dobot::DobotPose>("dobot", 1000);
    ros::Rate loop_rate(10);
    float input_x = 0.0, input_y = 0.0, input_z = 0.0, input_r = 0.0;

    while (ros::ok()) {
        dobot::DobotPose sendPose;
        cout << "Please input the relative pose coordinate to move dobot: x y z r" << endl;
        if (!(cin >> input_x >> input_y >> input_z >> input_r)) {
            cin.clear();
            cin.sync();
            cout << "Err: Input wrong value" << endl;
            return -1;
        }
        ROS_INFO("You input the coordinate ( %02f, %02f, %02f, %02f )", input_x, input_y, input_z, input_r);
        sendPose.x = input_x;
        sendPose.y = input_y;
        sendPose.z = input_z;
        sendPose.r = input_r;
        pub.publish(sendPose);
        ros::spinOnce();
    }
    return 0;
}
