#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "dobot/DobotPoseMsg.h"
#include "dobotDriver.hpp"

using namespace std;


void rosSetPoseCB(const dobot::DobotPoseMsg receivePose);

int main(int argc, char * argv[])
{
    // else if(!strcmp(argv[1], "Set2Zero")) {
        // DobotDriver dobot;
        // int ret = dobot.set2Zero();
        // if ( -1 == ret ) {
            // perror("fault to set to zero position");
        // }
    // }
    FullPose_t currentPose;
    int ret = 0;
    dobot::DobotPoseMsg pubPoseMsg;

    // Boot ros
    ros::init(argc, argv, "runDobot");
    ros::NodeHandle node;
    DobotDriver initDobot(node);
    ros::Rate loop_rate(10);

    while(ros::ok()) {
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

/**
 *  @function:  rosSetPoseCB
 *  @brief:     This is a callback function for ros subscriber(relative_pose)
 *              to receive position command
 *  @arg:      receivePose: is a DobotPoseMsg type argument, contain position command
 *  */
void DobotDriver::rosSetPoseCB(const dobot::DobotPoseMsg receivePose) {
    Pose_t poseCommand;
    poseCommand.x = receivePose.x;
    poseCommand.y = receivePose.y;
    poseCommand.z = receivePose.z;
    poseCommand.r = receivePose.r;
    int ret = runPointset(poseCommand);
    if ( -1 == ret ) {
        perror("fault to run Pointset method to dobot");
    }
    cout << "INFO: move dobot arm done ----- " << endl;
}
