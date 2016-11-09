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
    float x;
    float y;
    float z;
    float r;
    float j1;
    float j2;
    float j3;
    float j4;
    float j5;
}FullPose_t;

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

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char checkSum;
}CmdGetCurrentPose_t;

typedef struct {
    unsigned char header1;
    unsigned char header2;
    unsigned char len;
    unsigned char id;
    unsigned char ctrl;
    unsigned char x[4];
    unsigned char y[4];
    unsigned char z[4];
    unsigned char r[4];
    unsigned char j[5];
    unsigned char checkSum;
}CmdGetCurrentPoseRet_t;


class DobotDriver {
private:
    // It contain the current pose of dobot arm, will be updated by every motion of arm.
    Pose_t currentPose;

    // 2 arguments of uart communication to dobot arm.
    int uartFd;
    const char *uartPort;

    // Methods with uart communication to dobot arm.
    
    /**
     *  @func:  uartInit
     *  @brif:  initialization of uart to communicate with dobot arm.
     *  @args:  none
     *  @retn:  int: negative return is fault, 0 is done.
     *  */
    int uartInit();

    /**
     *  @func:  openPort
     *  @args:  none
     *  @retn:  none
     *  */
    void openPort();

    /**
     *  @func:  setUartOpt
     *  @args:  some arguments with uart communication: speed, bits, event, stop, etc.
     *  @retn:  none
     *  */
    void setUartOpt(const int nSpeed, const int nBits, const char nEvent, const int nStop);

    /****
     *  Methods about command last byte: Checksum.
     *  This byte in dobot arm command is working with Complement check, 
     *  which is the complement of Content part of command.
     *       Checksum = 0xFF - (cmd.id + cmd.ctrl + cmd.params) + 0x01
    ****/
    /**
     *  @func:  checkChecksum
     *  @brif:  check cs value in the received data to determin the validity of data.
     *  @args:  received command
     *  @retn:  check result: 0 is for right, negative value means wrong.
     *  */
    int checkChecksum(CmdPointSetRet_t cmd);

    /****
     *  Methods about create a command.
     *  Different command have different length and bits, be attention.
     *  
     * **/
    /**
     *  @func:  createPointsetCmd
     *  @brif:  create a command of move arm with PointSet method.
     *  @args:  pose is the PointSet required
     *  @retn:  CmdPointSet_t is the command it created.
     *  */
    CmdPointSet_t createPointsetCmd(Pose_t pose);

    /**
     *  @func:  sendPointsetCmd
     *  @brif:  send Pointset command to the dobot arm.
     *  @args:  cmd is the command waiting for send, pose is a assist args
     *  @retn:  none
     *  */
    void sendPointsetCmd(CmdPointSet_t cmd, Pose_t pose);

    CmdGetCurrentPose_t createGetCurrentPoseCmd();
    void sendGetCurrentPoseCmd(CmdGetCurrentPose_t);

    /**
     *  @func:  printPointsetCmd
     *  @brif:  print the command of Pointset method
     *  @args:  cmd is the command of Pointset
     *  @retn:  none
     *  */
    void printPointsetCmd(CmdPointSet_t cmd);

    /**
     *  @func:  printPointsetRetCmd
     *  @brif:  print the receive command of Pointset method
     *  @args:  cmd is the receive command 
     *  @retn:  none
     *  */
    void printPointsetRetCmd(CmdPointSetRet_t cmd);

    void printGetCurrentPoseCmd(CmdGetCurrentPose_t cmd);
    void printGetCurrentPoseRetCmd(CmdGetCurrentPoseRet_t cmd);

public:
    /**
     *  @func:  DobotDriver
     *  @brif:  construct methods
     *  @args:  none
     *  */
    DobotDriver();
    DobotDriver(Pose_t pose);

    /**
     *  @func:  runPointset
     *  @brif:  publish Pointset command to dobot arm to move it
     *  @args:  pose is a struct value with Pointset command
     *  @retn:  return a value to get correct running information of command
     *  */
    int runPointset(Pose_t pose);

    /**
     *  @func:  getCurrentPose
     *  @brif:  get current dobot arm FullPose
     *  @args:  none
     *  @retn:  return current pose of dobot arm
     *  */
    Pose_t getCurrentPose();

};

#endif /* ifndef DOBOT_DRIVER_H_ */
