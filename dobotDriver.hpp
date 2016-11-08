#ifndef DOBOT_DRIVER_H_
#define DOBOT_DRIVER_H_ value

#define HD      0xAA
#define CTRLR           0x00
#define CTRLW           0x01
#define CTRLQ           0x02
#define LEN_POINTSET    0x13
#define LEN_POINTSETRET 0x0A
#define ID_POINTSET     0x54

typedef struct {
    unsigned int mode;
    float x;
    float y;
    float z;
    float r;
}Pose_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;       // This command is always be write to dobot and isQueued
    unsigned char mode;
    unsigned char x[4];
    unsigned char y[4];
    unsigned char z[4];
    unsigned char r[4];
    unsigned char checkSum;
}CmdPointSet_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char queuedCmdIndex[8];
    unsigned char checkSum;
}CmdPointSetRet_t;


class DobotDriver {
private:
    Pose_t currentPose;
    int uartFd;
    const char *uartPort;

    int uartInit();
    void openPort();
    void setUartOpt(const int nSpeed, const int nBits, const char nEvent, const int nStop);

    unsigned char calculateChecksum(unsigned char *preCommand);
    int checkChecksum(CmdPointSetRet_t cmd);

    CmdPointSet_t createPointsetCmd(Pose_t pose);
    void sendPointsetCmd(CmdPointSet_t cmd, Pose_t pose);
    void printPointsetCmd(CmdPointSet_t cmd);
    void printPointsetRetCmd(CmdPointSetRet_t cmd);

public:
    DobotDriver();
    DobotDriver(Pose_t pose);
    int runPointset(Pose_t pose);

};

#endif /* ifndef DOBOT_DRIVER_H_ */
