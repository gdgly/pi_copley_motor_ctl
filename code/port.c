//port.c
#include "shiki.h"

//设置串口参数，波特率 奇偶位等
int set_opt(int fd,int nSpeed,int nBits,char nEvent,int nStop)
{
	struct termios newtio,oldtio;
	//保存现有串口参数设置
	if(tcgetattr(fd,&oldtio)!=0){
		perror("Setup Serial 1");
		return -1;
	}
	bzero(&newtio,sizeof(newtio));
	//设置该标志，才可以设置波特率，字符大小，数据位，停止位等参数
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//设置停止位
	switch(nBits){
		case 7:
			newtio.c_cflag |=CS7;
			break;
		case 8:
			newtio.c_cflag |=CS8;
			break;
	}
	//设置奇偶校验位
	switch(nEvent) {
		case 'O':	//奇
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK|ISTRIP);
		break;
		case 'E':	//偶
			newtio.c_iflag |= (INPCK|ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
		break;
		case 'N':	//无校验
			newtio.c_cflag &= ~PARENB;
		break;
	}
	//设置波特率
	switch(nSpeed){
		case 300:
			cfsetispeed(&newtio,B300);
			cfsetospeed(&newtio,B300);
		break;
		case 600:
			cfsetispeed(&newtio,B600);
			cfsetospeed(&newtio,B600);
		break;
		case 2400:
			cfsetispeed(&newtio,B2400);
			cfsetospeed(&newtio,B2400);
		break;
		case 4800:
			cfsetispeed(&newtio,B4800);
			cfsetospeed(&newtio,B4800);
		break;
		case 9600:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
		break;
		case 115200:
			cfsetispeed(&newtio,B115200);
			cfsetospeed(&newtio,B115200);
		break;
		default:
			cfsetispeed(&newtio,B9600);
			cfsetospeed(&newtio,B9600);
		break;
	}
	//设置停止位
	if(nStop ==1){
		newtio.c_cflag &= ~CSTOPB;
	}else if(nStop==2) {
		newtio.c_cflag |= CSTOPB;
	}
	//设置等待时间和最小接收字符，设置等待时间为1s
	newtio.c_cc[VTIME]=1;
	newtio.c_cc[VMIN]=0;
	//处理未接收字符
	tcflush(fd,TCIFLUSH);
  	//激活新配置
	if((tcsetattr(fd,TCSANOW,&newtio))!=0){
		perror("com set error");
		return -1;
	}
//	fprintf(stderr,"set done!\n");
	return 0;
}

//串口初始化，设置串口参数，驱动器上电默认波特率为9600，需要将其改为115200
void driver_init(int port,char* port_num)
{
	int ret=0;
	int nwrite;

	if(port_num != MOTOR_PORT_NUM){
 		ret = set_opt(port,115200,8,'N',1);
  		if(ret < 0){
			perror("set_opt error\n");
		}
		return;
	}

	if(port_num == MOTOR_PORT_NUM){
//		ret = set_opt(port,9600,8,'N',1);			
//		if(ret < 0) {
//			perror("set_opt error\n");
//		}
//		char baud[]="s r0x90 115200\n";
//		write(port,baud,strlen(baud));			//驱动器上电默认波特率为9600，需要将其改为115200
//
//		usleep(500000);

 		ret = set_opt(port,115200,8,'N',1);
  		if(ret < 0) {
			perror("set_opt error\n");
		}
		return;
	}
}



//打开串口设备函数
//驱动器串口默认为USB1，电位计串口默认为USB0，力传感器串口默认为USB2
int tty_init(char *CurrentPort)
{
	char com[16],pnum[2];
	int TempPort;
	//打开串口设备
//  	if (argc > 1) {
//  	  	CurrentPort = atoi(argv[1]);
//  	  	if(CurrentPort>255)
// 	  		CurrentPort=0;
//  	}

  memset(com, 0, sizeof(com));
//  	strcpy(com, "/dev/ttyUSB");
//  	itoa(CurrentPort, pnum, 10);
//	memset(pnum,0,sizeof(pnum));
//	sprintf(pnum,"%d",CurrentPort);
//  	strcat(com, pnum);
//  	fprintf(stderr,"port=%d,dev=%s\n",CurrentPort,com);
	sprintf(com,CurrentPort,NULL);
  TempPort = open(com, O_RDWR|O_NOCTTY|O_NDELAY);
	printf("tempport = %d\n",TempPort);
  if (TempPort < 0 ) {
		return TempPort;
  }
  	//恢复串口为阻塞状态
  if(fcntl(TempPort,F_SETFL,0)<0)
  	printf("fcntl failed\n");
  else
  	printf("fcntl=%d\n",fcntl(TempPort,F_SETFL,0));
  	//测试是否为终端设备
  if(isatty(STDIN_FILENO)==0){
  	printf("standard input is not a terminal device\n");
	}
  else
  	printf("isatty success!\n");

	return TempPort;
}


