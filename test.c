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



int ComPort,CurrentPort;
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
	newtio.c_cc[VTIME]=10;
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


void del_first_two(char*s)
{
	while(*s != '\0')
	{
	*s=*(s+2);
	s++;
	}
}


void del_chr(char *s,char ch)
{
	char *t=s;
	while(*s != '\0')
	{
		if(*s != ch)
			*t++=*s;
		s++;
	}
	*t='\0';
}

int is_in(char *s, char c)
{
	int k = 0;
	while(*s != '\0')
	{
		if(*s==c)
			{	
			return 1;
			}
		else
			{
			s++;
			}
	}
	return 0;
}



int rev2com()
{
	int32_t 		readcnt = 0,nread,ncheck=0;
	char			data[256],datagram[64];
	memset(data,'\0',256);
	while(1)
	{
		nread = read(ComPort,datagram,1);
		if(nread<0)
		{
			printf("read error nread=%d\n",readcnt);	
		}
		if(nread == 0)
		{
			return 0;
		}
		data[readcnt] = datagram[0];
		readcnt++;
		if(datagram[0]==0x0D)
		{
			memcpy(user_p.temp,data,sizeof(data));
			printf("rev = %s\n",user_p.temp);
//			if(ncheck = is_in(user_p.temp,*DRIVER_V))
//				{	
//					del_first_two(user_p.temp);
//				}
//			else if(ncheck = is_in(user_p.temp,*DRIVER_E))
//				{
//					del_first_two(user_p.temp);
//					printf("motor error = %s\n",user_p.temp);
//				}
//			else if(ncheck = is_in(user_p.temp,*DRIVER_OK))
//				{
//					
//				}
			return 1;	
		}
	}	
}



void driver_init()
{
	int ret=0;
	int nwrite;
  	//设置串口参数
//	ret = set_opt(ComPort,9600,8,'N',1);
//	if(ret < 0) {
//		perror("set_opt error\n");
//	}
//
//	char baud[]="s r0x90 115200\n";
//	nwrite = write(ComPort,baud,strlen(baud));
//	printf("nwrite=%d\n",nwrite);

//	usleep(500000);
//        printf("sadadasdada\n");
 	ret = set_opt(ComPort,115200,8,'N',1);
  	if(ret < 0) {
		perror("set_opt error\n");
	}
	
	sprintf(user_p.com,"s r0xc8 0\n");
	write(ComPort,user_p.com,strlen(user_p.com));
	rev2com(user_p);


}


void tty_init()
{
	char			com[16],pnum[2];

	//打开串口设备
	CurrentPort = 1;
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

  	ComPort = open(com, O_RDWR|O_NOCTTY);//|O_NDELAY
  	if (ComPort < 0 ) {
  		fprintf(stderr, "Open %s error!\n",com);
  		exit(1);
  	}
  	//恢复串口为阻塞状态
  	if(fcntl(ComPort,F_SETFL,0)<0)
  		printf("fcntl failed\n");
  	else
  		printf("fcntl=%d\n",fcntl(ComPort,F_SETFL,0));
  	//测试是否为终端设备
  	if(isatty(STDIN_FILENO)==0) {
  		printf("standard input is not a terminal device\n");
	}
  	else
  		printf("isatty success!\n");


}

int main()
{

	int nwrite,nread,t_set;
	char s[20];


	tty_init();
	driver_init();
	while(1)
	{
		while(1)
		{
		printf("enter command\n");
		fgets(s,20,stdin);
		sprintf(user_p.com,s);
		nwrite = write(ComPort,user_p.com,strlen(user_p.com));
		rev2com(user_p);
		}
	}
	close(ComPort)
}
