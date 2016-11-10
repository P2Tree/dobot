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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>

#include "dobotDriver.hpp"

using namespace std;

DobotDriver::DobotDriver() : uartPort("/dev/ttyUSB0"){
    if ( uartInit() )
        exit(0);
    FullPose_t absPose;
    // This is only to update currentPose variable, method will read current pose
    // and let it into currentPose
    getCurrentPose(absPose);
    cout << "init absolute pose is: ";
    cout << "( " << absPose.x << ", " << absPose.y << ", " << absPose.z << ", " << absPose.r << ")" << endl;
    //TODO: setZero
}

int DobotDriver::uartInit() {
    try{
        openPort();
        setUartOpt(115200, 8, 'N', 1);
    }
    catch(int i) {
        if (-1 == i) {
            perror("open uart port error");
        }
        if (-2 == i) {
            perror("set uart options error");
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

void DobotDriver::setZero() {
    //TODO: read file for zero
    zeroX = currentPose.x;
    zeroY = currentPose.y;
    zeroZ = currentPose.z;
    zeroR = currentPose.r;
}

void DobotDriver::updateZero() {
    // let world coordinate: currentPose write to file:zero
}

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
    cmd.mode = (unsigned char)pose.mode;
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
    // printPointsetCmd(cmd);
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
    // printGetCurrentPoseCmd(cmd);
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

    // printPointsetRetCmd(retData);
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

    // printGetCurrentPoseRetCmd(retData);
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
    cout << "current pose is: " << "(" << currentPose.x << ", " << currentPose.y;
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
    cout << "current pose is: " << "(" << currentPose.x << ", " << currentPose.y;
    cout << ", " << currentPose.z << ", " << currentPose.r << ")" << endl;
    return 0;
}


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
