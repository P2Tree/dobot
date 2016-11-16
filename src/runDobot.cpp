#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "dobot/DobotPoseMsg.h"
#include "dobotDriver.hpp"

using namespace std;

#define USAGE   cout << "Usage:" << endl; \
                cout << "     ./runDobot SetPose" << endl; \
                cout << "     ./runDobot GetPose" << endl; \
                cout << "     ./runDobot Set2Zero" << endl; \
                cout << "     ./runDobot UpdateZero" << endl; \

static Pose_t poseCommand={1, 0.0, 0.0, 0.0, 0.0};
static int moveF = 0;

void rosSetPoseCB(const dobot::DobotPoseMsg receivePose);


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
    FullPose_t currentPose;
    int ret = 0;
    dobot::DobotPoseMsg pubPoseMsg;
    ros::init(argc, argv, "runDobot");
    ros::NodeHandle node;
    DobotDriver initDobot(node);
    ros::Subscriber sub = node.subscribe<dobot::DobotPoseMsg>("dobot/relative_pose", 1000, rosSetPoseCB);
    ros::Publisher pub = node.advertise<dobot::DobotPoseMsg>("dobot/current_pose", 1000);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        if ( 1 == moveF ) {
            ret = initDobot.runPointset(poseCommand);
            if ( -1 == ret ) {
                perror("fault to run Pointset method to dobot");
                return -1;
            }
            cout << "INFO: move dobot arm done ----- " << endl;
            moveF = 0;
        }

        ret = initDobot.getCurrentPose(currentPose);
        if ( -1 == ret ) {
            perror("fault to get current dobot position");
            return -1;
        }
        pubPoseMsg.x = currentPose.x;
        pubPoseMsg.y = currentPose.y;
        pubPoseMsg.z = currentPose.z;
        pubPoseMsg.r = currentPose.r;
        pub.publish(pubPoseMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void rosSetPoseCB(const dobot::DobotPoseMsg receivePose) {
    poseCommand.x = receivePose.x;
    poseCommand.y = receivePose.y;
    poseCommand.z = receivePose.z;
    poseCommand.r = receivePose.r;
    moveF = 1;
}
