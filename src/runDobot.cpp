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
    // 
    // Boot ros
    ros::init(argc, argv, "runDobot");
    ros::NodeHandle node;
    DobotDriver initDobot(node);
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        initDobot.rosPublishPose();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

