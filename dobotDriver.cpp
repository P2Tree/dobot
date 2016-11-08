/*******************************************
 * 
 * @file    dobotDriver.c
 * @brief   dobot arm-robot driver file
 * @author  Yang Liuming <dicksonliuming@gmail.com>
 * @date    2016-11-07
 * 
 * ****************************************/

#include <iostream>
#include <stdio.h>
//#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>

#include "dobotDriver.hpp"

using namespace std;

DobotDriver::DobotDriver(){
    currentPose.mode = 0;
    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.z = 0.0;
    currentPose.r = 0.0;
}
DobotDriver::DobotDriver(Pose_t pose) {
    currentPose = pose;
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
    return 1;
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
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_iflag &= ~(INPCK | ISTRIP | IUCLC);

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
    printPointsetCmd(cmd);
    return cmd;
}

void DobotDriver::sendPointsetCmd(CmdPointSet_t cmd, Pose_t pose) {
    CmdPointSetRet_t retData;
    CmdPointSetRet_t *pRetData = &retData;
    int retLen = 0;
    CmdPointSet_t * pcmd = &cmd;
    tcflush(uartFd, TCOFLUSH);
    write(uartFd, pcmd, sizeof(cmd));
    sleep(2);
    retLen = read(uartFd, pRetData, 14);
    printf("receive data len: %d", retLen);
    printPointsetRetCmd(retData);
    int csCheck = checkChecksum(retData);
    tcflush(uartFd, TCIFLUSH);

    if (!(retLen > 0)) {
        throw -1;
        return;
    }
    if (0 != csCheck) {
        throw -2;
        return;
    }
    currentPose = pose;
    return;
}

int DobotDriver::checkChecksum(CmdPointSetRet_t retData) {
    return 0;   //check cs right
}

int DobotDriver::runPointset(Pose_t pose) {
    CmdPointSet_t cmd;
    cmd = createPointsetCmd(pose);
    try{
        sendPointsetCmd(cmd, pose);
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
    return 0;
}

void DobotDriver::printPointsetCmd(CmdPointSet_t cmd) {
    cout << "DEBUG: pointset command is:" << endl;
    cout << "HD HD LN ID CR MD PX PY PZ PR CS" << endl;
    cout << cmd.header1 << cmd.header2 << cmd.len << cmd.id << cmd.ctrl;
    cout << cmd.mode;
    for(int i=0; i<0; i++)
        cout << cmd.x[i];
    for(int i=0; i<0; i++)
        cout << cmd.y[i];
    for(int i=0; i<0; i++)
        cout << cmd.z[i];
    for(int i=0; i<0; i++)
        cout << cmd.r[i];
    cout << cmd.checkSum << endl;
}
 
void DobotDriver::printPointsetRetCmd(CmdPointSetRet_t cmd) {
    cout << "DEBUG: pointset receive command is:" << endl;
    cout << "HD HD LN ID CR Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 CS" << endl;
    cout << cmd.header1 << cmd.header2 << cmd.len << cmd.id << cmd.ctrl;
    cout << cmd.queuedCmdIndex[0] << cmd.queuedCmdIndex[1] << cmd.queuedCmdIndex[2];
    cout << cmd.queuedCmdIndex[3] << cmd.queuedCmdIndex[4] << cmd.queuedCmdIndex[5];
    cout << cmd.queuedCmdIndex[6] << cmd.queuedCmdIndex[7];
    cout << cmd.checkSum << endl;
}
