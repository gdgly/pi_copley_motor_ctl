#ifndef _SHIKI_HEADER
#define _SHIKI_HEADER

struct user_t{
	int32_t p0;
	int32_t p1;
	int32_t c;
	int32_t v;
	char com[256];
	char temp[256];
};

struct motor_ctl_t{
	char com[256];
	int32_t temp;
};

struct shiki_t{
	char g[16];
	char v[16];
	char e[16];
	char ok[16];
};

#define ADC_PORT_NUM 0
#define MOTOR_PORT_NUM 1
#define FORCE_PORT_NUM 2

#define DRIVER_G "g"
#define DRIVER_V "v"
#define DRIVER_E "e"
#define DRIVER_OK "ok"

#define SET_POSITION "s r0x32 %d\n"
#define GET_POSITION "g r0x32\n"
#define SET_MOTION "s r0xca %d\n"
#define SET_VELOCITY "s r0xcb %d\n"
#define GET_CURRENT "g r0x0c\n"
#define TRAJECTORY_ABORT "t 0\n"
#define TRAJECTORY_MOVE "t 1\n"
#define ABSOLUTE_MOVE 0
#define RELATIVE_MOVE 256
#define SET_MOVE_MODE "s r0xc8 %d\n"	//0 absolute move   256 relative move
#define DISABLE_MOTOR 0
#define ENABLE_POSITION_MODE 21
#define SET_DESIRED_STATE "s r0x24 %d\n"	//0 disable the move 
						//21 enable position mode
#define SET_BAUD_115200 "s r0x90 115200\n"
#define GET_MOTOR_FUALT	"g r0xa4\n"
#define CLEAR_MOTOR_FUALT "s r0xa4 %d\n"


#endif