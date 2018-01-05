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


struct motor_ctl_t motor_ctl(char *msg, void *para, int port);


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

void driver_init(int port,uint8_t port_num)
{
	int ret=0;
	int nwrite;

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


uint32_t get_force(int port)
{

	int32_t 		readcnt = 0,nread,ncheck=0,state = 0;
	uint8_t			data[256],datagram[64];
	float	  		adc_temp;	
	uint32_t 		force_temp;
	while(1){
		datagram[0] = 0;
		nread = read(port,datagram,1);
		if(nread<0){
			printf("read error nread=%d\n",readcnt);	
		}
		switch(state){
		case 0:
			if(datagram[0]==0x53){
				data[readcnt] = datagram[0];
				readcnt++;
				state=1;
			}			
			break;
		case 1:
			if(readcnt < 3){
				data[readcnt] = datagram[0];
				readcnt++;
				break;
			}
			else{
				data[readcnt] = datagram[0];
				if(data[3] == data[1]^data[2]){
					state=2;
					readcnt++;
					break;
				}
				else{
					printf("test3=%x\n",datagram[0]);
					state = 0;
					readcnt = 0;
					break;
				}
			}
		case 2:
			if(datagram[0]==0x59){
				data[readcnt] = datagram[0];
//				adc_temp = ((float)((data[2]<<8)|data[1]))/1000;
//				force_temp = ((uint32_t)(((data[2]<<8)|data[1])-600))/100;

				force_temp = (uint32_t)((data[2]<<8)|data[1]);
				if(force_temp > 350){
					force_temp = force_temp -350;
				}
				else{
					force_temp = 0;
				}
//				force_temp = (uint32_t)((data[2]<<8)|data[1]);
//				printf("temp = %f\n",adc_temp);
				
//				printf("temp = %u\n",force_temp);
				state = 0;
				readcnt= 0;
				return force_temp;
				break;
			}
			else{
//				printf("test4=%x\n",datagram[0]);
				state = 0;
				readcnt= 0;
				break;
			}
		}
	}
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
//			printf("rev = %s\n",temp.com);
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

uint32_t bubble_sort_and_average(void *msg, uint8_t len)
{
	uint32_t temp;
	uint32_t *s;
	int i,j,flag = 1;
	
	if(len < 8){
		return 0;
	}

	for(j=0;j<len-1&&flag;j++){
		s = msg;
		flag = 0;
		for(i=0;i<len-1-j;i++){
			if(*s > *(s +1)){
				temp = *s;
				*s = *(s+1);
				*(s+1) = temp;
				flag = 1;
			}
			s++;
		}
	}
	s = msg;
	s++;
	temp = 0;
	while((uint32_t*)s<(uint32_t*)msg+len-3){
//		printf("%d ",*s);
		temp = temp + *s;
		s ++;	
	}
//	printf("\n");
	temp = (uint32_t)(temp/(len-4));
	return temp;
}

void main()
{
	int AdcPort,MotorPort,FrocePort;
	float adc_temp,deltav_adc,pot_temp[12];
	int motor_temp,deltav_motor,motor_temp_p,motor_temp_v;
	char s[20];	
	int i,nwrite,index;
	uint32_t force_t,force_temp[12],force_command;
	uint32_t	time_now,time_mark;
	int32_t deltav_force,integral_force;
	struct timeval tv;

//	AdcPort = tty_init(ADC_PORT_NUM);
//	driver_init(AdcPort,ADC_PORT_NUM);

	FrocePort = tty_init(FORCE_PORT_NUM);
	driver_init(FrocePort,FORCE_PORT_NUM);


	MotorPort = tty_init(MOTOR_PORT_NUM);
	driver_init(MotorPort,MOTOR_PORT_NUM);

	gettimeofday(&tv,NULL);
	time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);	

	while(1){	

		printf("enter force\n");
		fgets(s,20,stdin);	
		sscanf(s,"%d",&force_command);
		integral_force = 0;
		while(1){

			tcflush(FrocePort,TCIFLUSH);
			struct motor_ctl_t motor_temp;
			motor_temp = motor_ctl(GET_POSITION,NULL,MotorPort);
			for(i=0;i<10;i++)
			{	
				force_temp[i] = get_force(FrocePort);	
			}

			force_t = bubble_sort_and_average(force_temp,10);
			gettimeofday(&tv,NULL);
			time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
			time_now = (time_now - time_mark)&0x000fffff;	
			printf("%u %d\n",time_now,force_t);
		
			deltav_force = force_command - force_t;
//			if((deltav_force > 200)||(deltav_force < -200)){
			if(abs(deltav_force) > 200){
				index = 0;
			}else{
				index = 1;
				integral_force = integral_force + deltav_force;
			}
			
			printf("integal = %d\n",integral_force);
			deltav_motor = deltav_force*1.5 + index*integral_force*0;
			motor_temp_p = motor_temp.temp - deltav_motor;

			motor_ctl(SET_MOTION,&motor_temp_p,MotorPort);
	
			motor_temp_v = 500000;
			motor_ctl(SET_VELOCITY,&motor_temp_v,MotorPort);

			motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
		}
	}
	close(FrocePort);
}
