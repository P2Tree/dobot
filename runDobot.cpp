#include <iostream>
#include <stdio.h>
#include <string.h>
#include "dobotDriver.hpp"

using namespace std;

#define USAGE   cout << "Usage:" << endl; \
                cout << "     ./runDobot SetPose" << endl; \
                cout << "     ./runDobot GetPose" << endl; \
                cout << "     ./runDobot UpdateZero" << endl;


int main(int argc, char * argv[])
{
    if (2 != argc) {
        USAGE
        return 0;
    }
    DobotDriver dobot;
    if (!strcmp(argv[1], "SetPose")) {
        // Pose_t pose={1, 130.9014, 0.0, 9.7602, 0.0};
        Pose_t pose={1, 0.0, 0.0, 0.0, 135.0};
        int ret = dobot.runPointset(pose);
        if ( -1 == ret ) {
            perror("fault to run Pointset method to dobot");
        }
        cout << "INFO: move dobot arm done ----- " << endl;
    }
    else if (!strcmp(argv[1], "GetPose")) {
        FullPose_t currentPose;
        int ret = dobot.getCurrentPose(currentPose);
        if ( -1 == ret ) {
            perror("fault to get current pose of dobot");
        }
        cout << "current x = " << currentPose.x << endl;
        cout << "current y = " << currentPose.y << endl;
        cout << "current z = " << currentPose.z << endl;
        cout << "current r = " << currentPose.r << endl;
        cout << "INFO: get current position done ----- " << endl;
    }
    else if(!strcmp(argv[1], "UpdateZero")) {
        int ret = dobot.updateZero();
        if ( -1 == ret ) {
            perror("fault to set zero for system");
        }
    }
    else {
        cout << "Input wrong argument" << endl;
        USAGE
    }

    return 0;
}

