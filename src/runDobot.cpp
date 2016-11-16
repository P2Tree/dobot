#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "dobot/DobotPose.h"
#include "dobotDriver.hpp"

using namespace std;

#define USAGE   cout << "Usage:" << endl; \
                cout << "     ./runDobot SetPose" << endl; \
                cout << "     ./runDobot GetPose" << endl; \
                cout << "     ./runDobot Set2Zero" << endl; \
                cout << "     ./runDobot UpdateZero" << endl; \

static Pose_t PoseCommand={1, 0.0, 0.0, 0.0, 0.0};

void rosSetPoseCB(const dobot::DobotPose);

int main(int argc, char * argv[])
{
    // if (2 != argc) {
        // USAGE
        // return 0;
    // }
    // if (!strcmp(argv[1], "SetPose")) {
        // // Pose_t pose={1, 130.9014, 0.0, 9.7602, 0.0};
        // DobotDriver dobot;
        // float input_x = 0.0, input_y = 0.0, input_z = 0.0, input_r = 0.0;
        // cout << "Please input pose value: (x y z r)" << endl;
        // cin >> input_x >> input_y >> input_z >> input_r;
        // Pose_t pose={1, 0.0, 0.0, 0.0, 0.0};
        // pose.x = input_x;
        // pose.y = input_y;
        // pose.z = input_z;
        // pose.r = input_r;
        // int ret = dobot.runPointset(pose);
        // if ( -1 == ret ) {
            // perror("fault to run Pointset method to dobot");
        // }
        // cout << "INFO: move dobot arm done ----- " << endl;
    // }
    // else if (!strcmp(argv[1], "GetPose")) {
        // DobotDriver dobot;
        // FullPose_t currentPose;
        // int ret = dobot.getCurrentPose(currentPose);
        // if ( -1 == ret ) {
            // perror("fault to get current pose of dobot");
        // }
        // cout << "current x = " << currentPose.x << endl;
        // cout << "current y = " << currentPose.y << endl;
        // cout << "current z = " << currentPose.z << endl;
        // cout << "current r = " << currentPose.r << endl;
        // cout << "INFO: get current position done ----- " << endl;
    // }
    // else if(!strcmp(argv[1], "Set2Zero")) {
        // DobotDriver dobot;
        // int ret = dobot.set2Zero();
        // if ( -1 == ret ) {
            // perror("fault to set to zero position");
        // }
    // }
    // else if(!strcmp(argv[1], "UpdateZero")) {
        // DobotDriver dobot;
        // int ret = dobot.updateZero();
        // if ( -1 == ret ) {
            // perror("fault to set zero for system");
        // }
    // }
    // else {
        ros::init(argc, argv, "runDobot");
        ros::NodeHandle node;
        DobotDriver initDobot(node);
        // ros::spinOnce();
    // }

    return 0;
}

