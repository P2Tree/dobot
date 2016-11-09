#include <iostream>
#include <stdio.h>
#include "dobotDriver.hpp"

int main()
{
    Pose_t pose={1, 130.9014, 0.0, 9.7602, 0.0};
    DobotDriver dobot;
    int ret = dobot.runPointset(pose);
    if ( -1 == ret ) {
        perror("fault to run Pointset function to dobot");
    }
    return 0;
}

