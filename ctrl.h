
#ifndef __CTRL_H__
#define __CTRL_H__

/*控制节点与1.电机控制通讯端口号为：8000；与2.步态检测通讯端口号为：8010*/
// #define MOTOSERVER_PORT   8000 
// #define GAITSERVER_PORT   8010 

#define NODEID_OF_CTRL    0x01
#define NODEID_OF_MOTOR   0x02
#define NODEID_OF_GAIT    0x03
#define NODEID_OF_POW     0x04

//控制消息类型
typedef enum 
{
    CTL_CMDINITIAL = 55,
    CTL_CMDPOWERDOWN,
    CTL_CMDMOTIONSTOP,
    CTL_CMDMOTIONSTART,
    CTL_CMDMOTIONSLEEP,
    CTL_CMDMOTIONSTANDBY,
} CTL_CMDTypeDef;

//返回消息类型
typedef enum 
{
 
  CTL_POWERON   = 66, //mcu
  CTL_POWERDOWN ,

  CTL_ERROR,
  CTL_DONE,

  CTL_READY,
  CTL_NOTREADY,

} CTL_StatusTypeDef;

typedef struct{
    uint16_t counter;
    int16_t  stop;
    int16_t  battery;
    int16_t  info1;
    int16_t  info2;
    int16_t  info3;
    int16_t  check;
}McuMsgTypeDef;


typedef struct{
    int16_t  nodeID;
    int16_t  opt;
    int16_t  reserve1;
    int16_t  reserve2;
    int16_t  reserve3; 
}CTLCmdMsgTypeDef;

typedef struct{
    int16_t  nodeID; //
    int16_t  rt;
    int16_t  reserve1;
    int16_t  reserve2;
    int16_t  reserve3; 
}CTLCmdRtMsgTypeDef;


#endif

