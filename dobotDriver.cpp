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
    currentPose.mode = 0;
    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.z = 0.0;
    currentPose.r = 0.0;
    if ( uartInit() )
        exit(0);
}
DobotDriver::DobotDriver(Pose_t pose) : uartPort("/dev/ttyUSB0"){
    currentPose = pose;
    if ( uartInit() )
        exit(0);
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

int DobotDriver::checkChecksum(CmdPointSetRet_t retData) {
    unsigned char addCS = 0x00;
    printf("retData[2] is: %d\n", (unsigned int)retData.len);
    for (unsigned int i=0; i<(unsigned int)retData.len; i++)
        i=i;
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
    if (!(retLen > 0)) {
        throw -1;
        return;
    }

    cout << "receive data len: " << oct << retLen << endl;
    printPointsetRetCmd(retData);
    int csCheck = checkChecksum(retData);
    tcflush(uartFd, TCIFLUSH);
    if (0 != csCheck) {
        throw -2;
        return;
    }

    currentPose = pose;
    return;
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
