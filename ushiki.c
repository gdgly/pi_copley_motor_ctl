#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <time.h>
#include <signal.h>
#include <sys/shm.h>
#include <asm/types.h>
#include <sys/ipc.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <stdint.h>
#include </home/pi/work/shiki.h>



struct user_t user_p;

int set_opt(int fd,int nSpeed,int nBits,char nEvent,int nStop)
{
	struct termios newtio,oldtio;
	//保存现有串口参数设置
	if(tcgetattr(fd,&oldtio)!=0) {
		perror("Setup Serial 1");
		return -1;
	}
	bzero(&newtio,sizeof(newtio));
	//设置该标志，才可以设置波特率，字符大小，数据位，停止位等参数
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//设置停止位
	switch(nBits) {
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
	switch(nSpeed) {
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
	if(nStop ==1) {
		newtio.c_cflag &= ~CSTOPB;
	}else if(nStop==2) {
		newtio.c_cflag |= CSTOPB;
	}
	//设置等待时间和最小接收字符
	newtio.c_cc[VTIME]=1;
	newtio.c_cc[VMIN]=0;
	//处理未接收字符
	tcflush(fd,TCIFLUSH);
  	//激活新配置
	if((tcsetattr(fd,TCSANOW,&newtio))!=0) {
		perror("com set error");
		return -1;
	}
//	fprintf(stderr,"set done!\n");
	return 0;
}


struct motor_ctl_t motor_ctl(char *msg, void *para, int port)
{
	int32_t readcnt = 0,nread,nwrite;
	char com[20];
	uint8_t data[256],datagram[64];
	int *p = para;
	struct motor_ctl_t temp;

	if(para == NULL){
		sprintf(com,msg);	
	}
	else{
		sprintf(com,msg,*p);
	}
	nwrite = write(port,com,strlen(com));
	
	memset(data,'\0',256);
	while(1)
	{
		nread = read(port,datagram,1);
		if(nread<0)
		{
			printf("read error nread=%d\n",readcnt);	
		}
		if(nread == 0)
		{
			printf("hello\n");
			return;
		}
		data[readcnt] = datagram[0];
		readcnt++;
		if(datagram[0]==0x0D)
		{
			memcpy(temp.com,data,sizeof(data));
			if(strstr(temp.com,"v")!=0)
				{	
					sscanf(temp.com,"%*s%d",&temp.temp);
					
				}
			else if(strstr(temp.com,"e")!=0)
				{
					sscanf(temp.com,"%*s%d",&temp.temp);
					printf("motor error = %d\n",temp.temp);
				}
			else if(strstr(temp.com,"ok")!=0)
				{
					
				}
			return temp;	
		}
	}

	return temp;
}



void driver_init(int port,uint8_t port_num)
{
	int ret=0;
	int nwrite;
  	//设置串口参数

	if(port_num == ADC_PORT_NUM){
 		ret = set_opt(port,115200,8,'N',1);
  		if(ret < 0) {
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
//		write(port,baud,strlen(baud));
//
//		usleep(500000);

 		ret = set_opt(port,115200,8,'N',1);
  		if(ret < 0) {
			perror("set_opt error\n");
		}
		return;
	}
}


int tty_init(int CurrentPort)
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
  	strcpy(com, "/dev/ttyUSB");
//  	itoa(CurrentPort, pnum, 10);
	memset(pnum,0,sizeof(pnum));
	sprintf(pnum,"%d",CurrentPort);
  	strcat(com, pnum);
  	fprintf(stderr,"port=%d,dev=%s\n",CurrentPort,com);

  	TempPort = open(com, O_RDWR|O_NOCTTY|O_NDELAY);
	printf("tempport = %d\n",TempPort);
  	if (TempPort < 0 ) {
  		fprintf(stderr, "Open %s error!\n",com);
  		exit(1);
  	}
  	//恢复串口为阻塞状态
  	if(fcntl(TempPort,F_SETFL,0)<0)
  		printf("fcntl failed\n");
  	else
  		printf("fcntl=%d\n",fcntl(TempPort,F_SETFL,0));
  	//测试是否为终端设备
  	if(isatty(STDIN_FILENO)==0) {
  		printf("standard input is not a terminal device\n");
	}
  	else
  		printf("isatty success!\n");

	return TempPort;
}

void clear_motor_fualt(int port)
{
	struct motor_ctl_t temp;
	temp = motor_ctl(GET_MOTOR_FUALT,NULL,port);
	
	motor_ctl(CLEAR_MOTOR_FUALT,&temp.temp,port);
	return;
}

void main()
{
	int MotorPort;
	int nwrite,nread,t_set;
	int cnt0=0,cnt1=1;
	int32_t  old_p,old_v;
	uint32_t time;
	struct timeval tv;
	struct motor_ctl_t motor_temp;

	MotorPort = tty_init(MOTOR_PORT_NUM);
	driver_init(MotorPort,MOTOR_PORT_NUM);

	clear_motor_fualt(MotorPort);

	nwrite = ENABLE_POSITION_MODE;
	motor_ctl(SET_DESIRED_STATE,&nwrite,MotorPort);

	nwrite = ABSOLUTE_MOVE;
	motor_ctl(SET_MOVE_MODE,&nwrite,MotorPort);


	t_set = 0;
	while(cnt0<1000)
	{
		cnt0++;
		if(cnt0 == 1000)
		{
			cnt0 = 0;
		}
		
		cnt1 = cnt0%100;
		if(cnt1<=37)
		{
			user_p.p0=18000;//20000
			user_p.v=500000;
		}
		else if(cnt1<=60)
		{
			user_p.p0=0;//35000
			user_p.v=1400000;
		}
		else if(cnt1<=100)
		{
			user_p.p0=25000;//0
			user_p.v=1400000;
		}
		if(user_p.p0 != old_p)
		{
			motor_ctl(SET_MOTION,&user_p.p0,MotorPort);
			t_set = 0;
		}
		old_p = user_p.p0;

		if(user_p.v != old_v)
		{
			motor_ctl(SET_VELOCITY,&user_p.v,MotorPort);
			t_set = 0;
		}	
		old_v = user_p.v;


		motor_temp = motor_ctl(GET_POSITION,NULL,MotorPort);
		user_p.p1 = motor_temp.temp;
		
		motor_temp = motor_ctl(GET_CURRENT,NULL,MotorPort);
		user_p.c = motor_temp.temp;
	
		if(t_set == 0)
		{
			motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
		}

		gettimeofday(&tv,NULL);
		time = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
		printf(" ti=%u cnt = %d po = %d ct = %d\n",time,cnt0,user_p.p1,user_p.c);
	}	
	close(MotorPort);
}