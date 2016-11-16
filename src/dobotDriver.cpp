/*******************************************
 * 
 * @file    dobotDriver.c
 * @brief   dobot arm-robot driver file
 * @author  Yang Liuming <dicksonliuming@gmail.com>
 * @date    2016-11-07
 * 
 * ****************************************/

#include <iostream>
#include <iomanip>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>

#include "ros/ros.h"
#include "dobot/DobotPose.h"
#include "dobotDriver.hpp"

using namespace std;

/**
 * 
 *  PRIVATE STATIC ARGUMENTS
 *
 * */
// zero arguments: zero point of software
float DobotDriver::zeroX = 200.0;
float DobotDriver::zeroY = 0.0;
float DobotDriver::zeroZ = -35;
float DobotDriver::zeroR = 0;

// clamp arguments: limited of space margin
float DobotDriver::MaxX = 300;
float DobotDriver::MinX = 200;
float DobotDriver::MaxY = 114;
float DobotDriver::MinY = -114;
float DobotDriver::MaxZ = 65;
float DobotDriver::MinZ = -35;
float DobotDriver::MaxR = 135;
float DobotDriver::MinR = -135;

DobotDriver* DobotDriver::pDobot = NULL;
/**
 * 
 *  PRIVATE METHODS
 *
 * */
DobotDriver::DobotDriver() : uartPort("/dev/ttyUSB0"){
    if ( uartInit() )
        exit(-2);
    // first set zero from zero.file file, after that [zero] should be setted
    // if (setZero()) {
        // perror("ERR: set zero values wrong");
        // exit(-1);
    // }
    // This is only to update currentPose variable, method will read current pose
    FullPose_t absPose;
    getCurrentPose(absPose);
    cout << "INFO: boot position(absolute): ";
    cout << "( " << absPose.x << ", " << absPose.y << ", " << absPose.z << ", " << absPose.r << ")" << endl;
}

DobotDriver::DobotDriver(ros::NodeHandle node) : uartPort("/dev/ttyUSB0"){
    // This member is important, current object is pointed by pDobot.
    // NOT SAFE TODO
    pDobot = this;

    if ( uartInit() )
        exit(-2);
    // first set zero from zero.file file, after that [zero] should be setted
    if (setZero(node)) {
        perror("ERR: set zero values wrong");
        exit(-1);
    }
    ros::Subscriber sub = node.subscribe("dobot", 1000, rosSetPoseCB);

    // This is only to update currentPose variable, method will read current pose
    // and let it into currentPose
    cout << "INFO: Setting zero position" << endl;
    set2Zero();
    sleep(1);
    FullPose_t absPose;
    getCurrentPose(absPose);
    cout << "INFO: boot position(absolute): ";
    cout << "( " << absPose.x << ", " << absPose.y << ", " << absPose.z << ", " << absPose.r << ")" << endl;
    cout << "INFO: boot done ------ " << endl;
}

int DobotDriver::uartInit() {
    try{
        openPort();
        setUartOpt(115200, 8, 'N', 1);
    }
    catch(int i) {
        if (-1 == i) {
            perror("ERR: open uart port error");
        }
        if (-2 == i) {
            perror("ERR: set uart options error");
        }
        return i;
    }
    return 0;
}

void DobotDriver::openPort() {
    if(-1 == (uartFd = open(uartPort, O_RDWR | O_NOCTTY | O_NONBLOCK)))
        throw -1;
}

void DobotDriver::setUartOpt(const int nSpeed, const int nBits, const char nEvent, const int nStop) {
    struct termios newtio, oldtio;
    if(tcgetattr(uartFd, &oldtio) != 0) {
        throw -2;
    }
    newtio.c_cflag = 0;
    newtio.c_iflag = 0;
    newtio.c_lflag = 0;
    newtio.c_oflag = 0;

    newtio.c_cflag |= CLOCAL | CREAD;

    switch(nBits) {
        case 7:
            newtio.c_cflag &= ~CSIZE;
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag &= ~CSIZE;
            newtio.c_cflag |= CS8;
            break;
        default:
            throw -2;
            return;
    }

    switch(nEvent) {
        case 'O':
            newtio.c_cflag &= ~PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            throw -2;
            return;
    }

    switch(nSpeed) {
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            throw -2;
            return;
    }

    if (1 == nStop)
        newtio.c_cflag &= ~CSTOPB;
    else if(2 == nStop)
        newtio.c_cflag |= CSTOPB;
    
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(uartFd, TCIOFLUSH);
    if ((tcsetattr(uartFd, TCSANOW, &newtio)) != 0) {
        tcsetattr(uartFd, TCSANOW, &oldtio);
        throw -2;
        return;
    }
    tcflush(uartFd, TCIOFLUSH);
}

// get zero value and put it into zero arguments
int DobotDriver::setZero(ros::NodeHandle node) {
    if (!node.getParam("/runDobot/zeroX", zeroX)) {
        cout << "ERR: wrong to get zero values from rosparams" << endl;
        return -1;
    }

    cout << "INFO: reset zero value from zero.file" << endl;
    cout << "INFO: current zero: ";
    cout << "(" << zeroX << ", " << zeroY << ", ";
    cout << zeroZ << ", " << zeroR << ")" << endl;
    // ifstream zerofile;
    // // ZEROFILE
    // zerofile.open("~/driver/zero.file", ios::in);     // open file for read
    // if (!zerofile.is_open()) {
        // cout << "ERR: open file zero.file for read, fail" << endl;
        // return -1;
    // }
    // zerofile >> zeroX >> zeroY >> zeroZ >> zeroR;
    // cout << "INFO: reset zero value from zero.file" << endl;
    // cout << "INFO: current zero: ";
    // cout << "(" << zeroX << ", " << zeroY << ", ";
    // cout << zeroZ << ", " << zeroR << ")" << endl;
    // zerofile.close();
    return 0;
}
// By the word, method: updateZero is the public method, and its definition in the below of file


int DobotDriver::checkChecksum( unsigned char *data, unsigned int datalen) {
    unsigned char addCS = 0x00;
    data = data + 3;
    for (unsigned int i=0; i<datalen-4; i++) {
        addCS += *data;
        data++;
    }
    addCS = 0xFF - addCS + 0x01;
    if (addCS != *data) {
        cout << "received data CS byte is: " << hex << (unsigned int)addCS << endl;
        return -1;      // check cs wrong
    }
    return 0;   //check cs right
}

CmdPointSet_t DobotDriver::createPointsetCmd(Pose_t pose) {
    CmdPointSet_t cmd;
    cmd.header1 = HD;
    cmd.header2 = HD;
    cmd.id = ID_POINTSET;
    cmd.len = LEN_POINTSET;
    cmd.ctrl = CTRLW | CTRLQ;
    // cmd.mode = pose.mode;
    cmd.mode = 0x01;
    char *pchar = (char *)&pose.x;
    for(int i=0; i<4; i++) {
        cmd.x[i] = *pchar;
        pchar++;
    }
    pchar = (char *)&pose.y;
    for(int i=0; i<4; i++) {
        cmd.y[i] = *pchar;
        pchar++;
    }
    pchar = (char *)&pose.z;
    for(int i=0; i<4; i++) {
        cmd.z[i] = *pchar;
        pchar++;
    }
    pchar = (char *)&pose.r;
    for(int i=0; i<4; i++) {
        cmd.r[i] = *pchar;
        pchar++;
    }

    char check = cmd.id + cmd.ctrl + cmd.mode;
    check += cmd.x[0] + cmd.x[1] + cmd.x[2] + cmd.x[3];
    check += cmd.y[0] + cmd.y[1] + cmd.y[2] + cmd.y[3];
    check += cmd.z[0] + cmd.z[1] + cmd.z[2] + cmd.z[3];
    check += cmd.r[0] + cmd.r[1] + cmd.r[2] + cmd.r[3];
    check = 0xFF - check + 0x01;
    cmd.checkSum = check;
    //printPointsetCmd(cmd);
    return cmd;
}

CmdGetCurrentPose_t DobotDriver::createGetCurrentPoseCmd() {
    CmdGetCurrentPose_t cmd;
    cmd.header1 = HD;
    cmd.header2 = HD;
    cmd.len = LEN_GETCURRENTPOSE;
    cmd.id = ID_GETCURRENTPOSE;
    cmd.ctrl = CTRLR;
    char check = cmd.id + cmd.ctrl;
    check = 0xFF - check + 0x01;
    cmd.checkSum = check;
    //printGetCurrentPoseCmd(cmd);
    return cmd;
}

void DobotDriver::sendPointsetCmd(CmdPointSet_t cmd) {
    CmdPointSetRet_t retData;
    CmdPointSetRet_t *pRetData = &retData;
    int retLen = 0;
    CmdPointSet_t * pcmd = &cmd;
    tcflush(uartFd, TCOFLUSH);
    write(uartFd, pcmd, sizeof(cmd));
    sleep(1);
    retLen = read(uartFd, pRetData, 20);
    if (!(retLen > 0)) {
        throw -1;
        return;
    }
    //printPointsetRetCmd(retData);
    tcflush(uartFd, TCIFLUSH);
    if ( -1 == checkChecksum((unsigned char*)pRetData, retLen) ) {
        throw -2;
        return;
    }

    return;
}

void DobotDriver::sendGetCurrentPoseCmd(CmdGetCurrentPose_t cmd, FullPose_t &retPose) {
    CmdGetCurrentPoseRet_t retData;
    CmdGetCurrentPoseRet_t *pRetData = &retData;
    int retLen = 0;
    CmdGetCurrentPose_t *pcmd = &cmd;
    tcflush(uartFd, TCOFLUSH);
    write(uartFd, pcmd, sizeof(cmd));
    sleep(1);
    retLen = read(uartFd, pRetData, 50);
    if (!(retLen > 0)) {
        throw -1;
        return;
    }

    //printGetCurrentPoseRetCmd(retData);
    tcflush(uartFd, TCIFLUSH);
    if ( -1 == checkChecksum((unsigned char*)pRetData, retLen) ) {
        throw -2;
        return;
    }

    retPose.x = *(float *)retData.x;
    retPose.y = *(float *)retData.y;
    retPose.z = *(float *)retData.z;
    retPose.r = *(float *)retData.r;
    retPose.j1 = *(float *)retData.j1;
    retPose.j2 = *(float *)retData.j2;
    retPose.j3 = *(float *)retData.j3;
    retPose.j4 = *(float *)retData.j4;
    retPose.j5 = *(float *)retData.j5;

}

void DobotDriver::updateCurrentPose(Pose_t pose) {
    currentPose = pose;
}

/**
 * 
 *  PUBLIC METHODS
 *  
 *  */

int DobotDriver::runPointset(Pose_t pose) {
    CmdPointSet_t cmd;

    // transfer relative coordinate to absolute coordinate
    // pose is the relative corrdinate
    Pose_t absPose;
    absPose.x = pose.x + currentPose.x;
    absPose.y = pose.y + currentPose.y;
    absPose.z = pose.z + currentPose.z;
    absPose.r = pose.r + currentPose.r;

    // decide limited space of margin
    if (absPose.x < MinX)
        absPose.x = MinX;
    else if (absPose.x > MaxX)
        absPose.x = MaxX;
    if (absPose.y < MinY)
        absPose.y = MinY;
    else if(absPose.y > MaxY)
        absPose.y = MaxY;
    if (absPose.z < MinZ)
        absPose.z = MinZ;
    else if(absPose.z > MaxZ)
        absPose.z = MaxZ;
    if (absPose.r < MinR)
        absPose.r = MinR;
    else if(absPose.r > MaxR)
        absPose.r = MaxR;
    // absPose is the world coordinate
    // pose is the relative coordinate
    cmd = createPointsetCmd(absPose);
    try{
        sendPointsetCmd(cmd);
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            return -1;
        }
        if (-2 == i) {
            perror("recevie data cs check wrong");
            return -1;
        }
    }
    updateCurrentPose(absPose);
    cout << "DEBUG: currentPose(world): " << "(" << currentPose.x << ", " << currentPose.y;
    cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
    return 0;
}

int DobotDriver::getCurrentPose(FullPose_t &retPose) {
    CmdGetCurrentPose_t cmd;
    Pose_t mPose;
    cmd = createGetCurrentPoseCmd();
    try {
        sendGetCurrentPoseCmd(cmd, retPose);
        // retPose is the world coordinate, return from dobot arm
        // transfer FullPose_t struct variable to Pose_t struct variable
        mPose.x = retPose.x;
        mPose.y = retPose.y;
        mPose.z = retPose.z;
        mPose.r = retPose.r;
        // Add return pose position with software zero calibration
        retPose.x -= zeroX;
        retPose.y -= zeroY;
        retPose.z -= zeroZ;
        retPose.r -= zeroR;
        // now retPose is the absolute coordinate
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            return -1;
        }
        if (-2 == i) {
            perror("receive data cs check wrong");
            return -1;
        }
    }
    updateCurrentPose(mPose);
    cout << "DEBUG: currentPose(world): " << "(" << currentPose.x << ", " << currentPose.y;
    cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
    return 0;
}

// This is a command of runDobot, to let arm return ro zero position
int DobotDriver::set2Zero() {
    CmdPointSet_t cmd;

    Pose_t absPose;
    cout << zeroX << " " << zeroY << " " << zeroZ << " " << zeroR << endl;
    absPose.x = zeroX;
    absPose.y = zeroY;
    absPose.z = zeroZ;
    absPose.r = zeroR;
    cmd = createPointsetCmd(absPose);
    try{
        sendPointsetCmd(cmd);
    }
    catch(int i) {
        if (-1 == i) {
            perror("receive no data");
            return -1;
        }
        if (-2 == i) {
            perror("recevie data cs check wrong");
            return -1;
        }
    }
    updateCurrentPose(absPose);
    cout << "DEBUG: currentPose(world): " << "(" << currentPose.x << ", " << currentPose.y;
    cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
    return 0;

}

int DobotDriver::updateZero() {
    ofstream zerofile;
    // ZEROFILE
    zerofile.open("~/driver/zero.file", ios::out);   // open file for write
    if (! zerofile.is_open()) {
        cout << "ERR: open file zero.file fail" << endl;
        return -1;
    }
    zerofile << currentPose.x << '\t';
    zerofile << currentPose.y << '\t';
    zerofile << currentPose.z << '\t';
    zerofile << currentPose.r << '\t';
    cout << "INFO: updated zero position into zero.file" << endl;
    zerofile.close();
    return 0;
}

/**
 * 
 *  CALLBACK PUBLISH METHODS
 *  
 *  */
//static
void DobotDriver::rosSetPoseCB(const dobot::DobotPose receivePose) {
    ROS_INFO("I heared: ( %02f, %02f, %02f, %02f )", receivePose.x, receivePose.y, receivePose.z, receivePose.r);
    Pose_t poseCommand;
    poseCommand.x = receivePose.x;
    poseCommand.y = receivePose.y;
    poseCommand.z = receivePose.z;
    poseCommand.r = receivePose.r;
    int ret = pDobot->runPointset(poseCommand);
    if ( -1 == ret ) {
        perror("fault to run Pointset method to dobot");
    }
    cout << "INFO: move dobot arm done ----- " << endl;
}
/**
 *
 *  PRIVATE METHODS of print
 *
 * */
void DobotDriver::printPointsetCmd(CmdPointSet_t cmd) {
    cout << "DEBUG: pointset command is:" << endl;
    cout << "HD HD LN ID CR MD X1 X2 X3 X4 Y1 Y2 Y3 Y4 Z1 Z2 Z3 Z4 R1 R2 R3 R4 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.mode << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[i] << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[i] << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[i] << " ";
    for(int i=0; i<4; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[i] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << endl;
 //   cout.unsetf(ios::hex);
}
 
void DobotDriver::printPointsetRetCmd(CmdPointSetRet_t cmd) {
    cout << "DEBUG: pointset receive command is:" << endl;
    cout << "HD HD LN ID CR Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    for (int i=0; i<8; i++)
        cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.queuedCmdIndex[i] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << endl;
}

void DobotDriver::printGetCurrentPoseCmd(CmdGetCurrentPose_t cmd) {
    cout << "DEBUG: getCurrentPose command is:" << endl;
    cout << "HD HD LN ID CR CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << endl;
}

void DobotDriver::printGetCurrentPoseRetCmd(CmdGetCurrentPoseRet_t cmd) {
    cout << "DEBUG: getCurrentPose receive command is:" << endl;
    cout << "HD HD LN ID CR X1 X2 X3 X4 Y1 Y2 Y3 Y4 Z1 Z2 Z3 Z4 R1 R2 R3 R4";
    cout << " M1 M2 M3 M4 N1 N2 N3 N4 S1 S2 S3 S4 L1 L2 L3 L4 U1 U2 U3 U4 CS" << endl;
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header1 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.header2 << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.len << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.id << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.ctrl << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.x[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.y[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.z[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.r[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j1[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j2[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j3[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j4[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[0] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[1] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[2] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.j5[3] << " ";
    cout << setfill('0') << setw(2) << hex << (unsigned int)cmd.checkSum << " " << endl;
}
