//zeromq_pull_fixed_position.c
#include "shiki.h"


#define MOTOR_PARA_DEFAULTS {	550,\
														 	8000,\
														 	25000,\
														 	18000,\
														 	800000,\
														 	500000,\
														 	0,\
														 	6.0,\
														 	0.1,\
														 	500,\
														 	-500\
														}
														
														
struct motor_ctl_t motor_ctl(char *msg, void *para, int port);

void thread_force_port(void);
void thread_pot_port(void);
void thread_motor_port(void);
void thread_gait_zeromq(void);
int thread_client_zeromq(void);
void thread_motor_module_run_info(void);

pthread_mutex_t mutex_force;
pthread_mutex_t mutex_pot;
pthread_mutex_t mutex_gait_msg;
pthread_mutex_t mutex_client_msg;
pthread_mutex_t mutex_run_info;

sem_t sem_client;
sem_t sem_motor;


uint32_t force_now;										//串口上报的力传感器值
float pot_now;
uint32_t state_now;										//socket收到的电机运动状态命令
int32_t motor_en_flag;										//1:工作   0:不工作   通过互斥锁操作
struct motor_module_run_info_t motor_module_run_info;
struct motor_para_init_t motor_para_init = MOTOR_PARA_DEFAULTS;

//设置串口参数，波特率 奇偶位等
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
	//设置等待时间和最小接收字符，设置等待时间为1s
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

//串口初始化，设置串口参数，驱动器上电默认波特率为9600，需要将其改为115200
void driver_init(int port,char* port_num)
{
	int ret=0;
	int nwrite;

	if(port_num != MOTOR_PORT_NUM){
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


//获取力传感器数据，一帧的格式为 帧头：0x53 数据：0x** 0x** 校验：两个数据的与 帧尾：0x59
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
			exit(1);	
		}
		
		if(nread == 0){					
			return 10.0;
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
					
					state = 0;
					readcnt = 0;
					
					pthread_mutex_lock(&mutex_run_info);			
      		motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
     			pthread_mutex_unlock(&mutex_run_info);					
     			
					break;
				}
			}
		case 2:
			if(datagram[0]==0x59){
				data[readcnt] = datagram[0];
//				adc_temp = ((float)((data[2]<<8)|data[1]))/1000;
//				force_temp = ((uint32_t)(((data[2]<<8)|data[1])-600))/100;

				force_temp = (uint32_t)((data[2]<<8)|data[1]);
				if(force_temp > 100){
					force_temp = force_temp -100;		//减100是为了将力传感器的数据最小值矫正到0
				}
				else{
					force_temp = 0;
				}
//				force_temp = (uint32_t)((data[2]<<8)|data[1]);
				state = 0;
				readcnt= 0;
				return force_temp;
				break;
			}
			else{

				state = 0;
				readcnt= 0;
				
				pthread_mutex_lock(&mutex_run_info);			
      	motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
     		pthread_mutex_unlock(&mutex_run_info);	
     			
				break;
			}
		}
	}
}

float get_pot(int port)
{

	int32_t 		readcnt = 0,nread,ncheck=0,state = 0;
	uint8_t			data[256],datagram[64];
	float	  		adc_temp;	
	while(1){
		datagram[0] = 0;
		nread = read(port,datagram,1);
		
		if(nread<0){
			exit(1);
		}
		
		if(nread == 0){	
			return 0xFFFF;
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
					state = 0;
					readcnt = 0;
					
					pthread_mutex_lock(&mutex_run_info);			
      		motor_module_run_info.pot_senser_error++;		//校验错误，收集错误信息
     			pthread_mutex_unlock(&mutex_run_info);
					
					break;
				}
			}
		case 2:
			if(datagram[0]==0x59){
				data[readcnt] = datagram[0];
				adc_temp = ((float)((data[2]<<8)|data[1]))/1000;					
				state = 0;
				readcnt= 0;
				return adc_temp;
				break;
			}
			else{
				state = 0;
				readcnt= 0;
				
				pthread_mutex_lock(&mutex_run_info);			
      	motor_module_run_info.pot_senser_error++;		//校验错误，收集错误信息
     		pthread_mutex_unlock(&mutex_run_info);
     							
				break;
			}
		}
	}
}


//驱动电机的接口，通过串口ASCII码的格式与驱动器通讯，通讯分两种
//一种是带有参数的，需要将参数传入到para
//不带参数的para传入NULL即可
//shiki.h文件里有所需要的通讯格式的宏定义
//返回为驱动器的反馈（v ***; ok; e **;）
struct motor_ctl_t motor_ctl(char *msg, void *para, int port)
{
	int32_t readcnt = 0,nread,nwrite;
	char com[20];
	uint8_t data[256],datagram[64];
	int *p = para;
	struct motor_ctl_t temp;

	if(para == NULL){
		sprintf(com,msg,NULL);	
	}
	else{
		sprintf(com,msg,*p);
	}
	nwrite = write(port,com,strlen(com));		//发送串口命令
	
	memset(data,'\0',256);
	while(1)
	{
		nread = read(port,datagram,1);	//获取串口命令
		if(nread<0)
		{
			printf("read error nread=%d\n",readcnt);
			exit(1);		
		}
		if(nread == 0)
		{
			printf("Communication fail\n");
			
			pthread_mutex_lock(&mutex_run_info);			
      motor_module_run_info.motor_driver_error++;		//通讯失败错误信息累计
     	pthread_mutex_unlock(&mutex_run_info); 
     	     			 	
			temp.state = -1;
			return temp;
		}
		data[readcnt] = datagram[0];
		readcnt++;
		if(datagram[0]==0x0D)	//最后一位为“\n”判断接收到最后一位后再处理数据
		{
			memcpy(temp.com,data,sizeof(data));
//			printf("rev = %s\n",temp.com);
			if(strstr(temp.com,"v")!=0)
				{	
					temp.state = 0;
					sscanf(temp.com,"%*s%d",&temp.temp);
					
				}
			else if(strstr(temp.com,"e")!=0)
				{
					sscanf(temp.com,"%*s%d",&temp.state);
					printf("motor error = %d\n",temp.state);
					
					pthread_mutex_lock(&mutex_run_info);			
      		motor_module_run_info.motor_driver_error++;		//通讯失败错误信息累计
     			pthread_mutex_unlock(&mutex_run_info);
     			
				}
			else if(strstr(temp.com,"ok")!=0)
				{
					
				}
			return temp;	
		}
	}
}

//对力传感器的数据进行处理
//冒泡算法+平均，除去了最大和最小的一个数
uint32_t bubble_sort_and_average(void *msg, uint8_t len)
{
	uint32_t temp;
	uint32_t *s;
	int i,j,flag = 1;
	
	if(len < 3){
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
	while((uint32_t*)s<(uint32_t*)msg+len-1){
		temp = temp + *s;
		s ++;	
	}
	temp = (uint32_t)(temp/(len-2));
	return temp;
}


void main()
{
/*	int AdcPort,MotorPort,FrocePort;
	float adc_temp,deltav_adc,pot_temp[12];
	int motor_temp,deltav_motor,motor_cmd_position,motor_cmd_velocity,pid_umax,pid_umin;
	char s[20];	
	int i,nwrite,index;
	uint32_t force_t,force_temp[12],force_command;
	uint32_t	time_now,time_mark;
	int32_t deltav_force,integral_force;
	struct timeval tv;*/


	pthread_t tid1,tid2,tid3,tid4,tid5,tid6;		//定义了四个线程
	
	pthread_mutex_init(&mutex_force,NULL);			//定义了三个互斥锁
	pthread_mutex_init(&mutex_pot,NULL);
	pthread_mutex_init(&mutex_gait_msg,NULL);
	pthread_mutex_init(&mutex_client_msg,NULL);
	pthread_mutex_init(&mutex_run_info,NULL);
	
	
	

	pthread_create(&tid1,NULL,(void*)thread_force_port,NULL);			//创建线程
	pthread_create(&tid2,NULL,(void*)thread_motor_port,NULL);
	pthread_create(&tid3,NULL,(void*)thread_gait_zeromq,NULL);
	pthread_create(&tid4,NULL,(void*)thread_client_zeromq,NULL);
	pthread_create(&tid5,NULL,(void*)thread_pot_port,NULL);
	pthread_create(&tid6,NULL,(void*)thread_motor_module_run_info,NULL);
	
	sem_init(&sem_client,0,0);
	sem_init(&sem_motor,0,0);
		
	
//	pthread_join(tid1,NULL);			//等待线程结束，每个线程都是while(1)循环，所以不会结束
//	pthread_join(tid2,NULL);
//	pthread_join(tid3,NULL);
	if((pthread_join(tid1,NULL) == 0)\
		||(pthread_join(tid2,NULL) == 0)\
		||(pthread_join(tid3,NULL) == 0)\
		||(pthread_join(tid4,NULL) == 0)\
		||(pthread_join(tid5,NULL) == 0)\
		||(pthread_join(tid6,NULL) == 0)\
		
		){
		
	}
}


//力传感器读取线程
void thread_force_port(void)
{
	int FrocePort;			//力传感器的端口号
	int i;
	uint32_t force_t,force_temp[12];
	
	FrocePort = tty_init(FORCE_PORT_NUM);
	driver_init(FrocePort,FORCE_PORT_NUM);	
	
	while(1){
		
		tcflush(FrocePort,TCIFLUSH);					//清除掉串口缓存，不然串口会缓存过多数据，导致实时性降低
		
		for(i=0;i<5;i++){	
			force_temp[i] = get_force(FrocePort);	
			if(force_temp[i] == 0xffff){
				pthread_mutex_lock(&mutex_run_info);			
				motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
				pthread_mutex_unlock(&mutex_run_info);					
				i--;
			}	
		}
		force_t = bubble_sort_and_average(force_temp,5);	
		
		pthread_mutex_lock(&mutex_force);			//互斥锁
		force_now = force_t;
		pthread_mutex_unlock(&mutex_force);
	}
	
	close(FrocePort);
}


//电位计读取线程
void thread_pot_port(void)
{
	int AdcPort;			//电位计的端口号
	int i;
	float pot_t,pot_temp[12];
	
	AdcPort = tty_init(ADC_PORT_NUM);
	driver_init(AdcPort,ADC_PORT_NUM);	
	
	while(1){
		
		tcflush(AdcPort,TCIFLUSH);					//清除掉串口缓存，不然串口会缓存过多数据，导致实时性降低
		
		for(i=0;i<10;i++)
		{	
			pot_temp[i] = get_pot(AdcPort);
			if((i>0)&(i<9)){
				pot_t = pot_temp[i]+pot_t ;
				if(pot_temp[i]==10.0){
					pthread_mutex_lock(&mutex_run_info);			
					motor_module_run_info.pot_senser_error++;		//校验错误，收集错误信息
					pthread_mutex_unlock(&mutex_run_info);	
					i--;	
				}
			}		
		}
		pot_t = pot_t /8;
		
		pthread_mutex_lock(&mutex_pot);			//互斥锁
		pot_now = pot_t;
		pthread_mutex_unlock(&mutex_pot);
	}
	
	close(AdcPort);
}



//电机控制线程
void thread_motor_port(void)
{
	int MotorPort;			
	int deltav_motor,deltav_motor_old,motor_cmd_position,motor_cmd_velocity,max_position;
//	char s[20];	
	int i,nwrite,index,nset_acc,max_force_cnt;
	uint32_t state_temp,state_old,gait_temp,force_temp,max_force;
	uint32_t time_now,time_mark;
	int32_t deltav_force;
	struct timeval tv;
	struct motor_ctl_t motor_position,motor_state,motor_speed;
	int32_t motor_en_flag_temp,motor_en_flag_temp_old;
	struct motor_para_init_t motor_para_init_temp;

	MotorPort = tty_init(MOTOR_PORT_NUM);
	driver_init(MotorPort,MOTOR_PORT_NUM);
	
	while(1){

	//		printf("enter force\n");
	//		fgets(s,20,stdin);	
	//		sscanf(s,"%d",&force_command);
		motor_para_init_temp = motor_para_init;
		max_position = motor_para_init_temp.max_position;
		deltav_motor_old = 0;			
		max_force_cnt = 0;
		
		gettimeofday(&tv,NULL);
		time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);		//获取系统时间，单位为ms


		sem_wait(&sem_motor);
		printf("this is a test start\n");
			
		motor_cmd_velocity = motor_para_init_temp.max_velocity;																		//设置运动速度为14000rpm 此参数需要可以配置
		motor_ctl(SET_VELOCITY,&motor_cmd_velocity,MotorPort);					

		nset_acc = motor_para_init_temp.nset_acc;					//驱动器的最大加速度设置参数
		motor_ctl(SET_MAX_DEC,&nset_acc,MotorPort);
		nset_acc = motor_para_init_temp.nset_acc;
		motor_ctl(SET_MAX_ACC,&nset_acc,MotorPort);

		while(1){
				
			pthread_mutex_lock(&mutex_client_msg);			//获取当前使能状态									
			motor_en_flag_temp = motor_en_flag;
			pthread_mutex_unlock(&mutex_client_msg);		
			motor_en_flag_temp = CTL_CMDMOTIONSTART;
				
			switch(motor_en_flag_temp){			
				case CTL_CMDINITIAL:																				//自检
					
					
				break;
				
				case CTL_CMDPOWERDOWN:																				//关机
					
				nwrite = DISABLE_MOTOR;
				motor_ctl(SET_DESIRED_STATE,&nwrite,MotorPort);
				return;
					
				break;
							
				case CTL_CMDMOTIONSLEEP:																		//停机
					
				if(motor_en_flag_temp != motor_en_flag_temp_old){	
					motor_cmd_position = motor_para_init_temp.zero_position;
					motor_ctl(SET_MOTION,&motor_cmd_position,MotorPort);
					motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);				
				}
					
				break;
						
				case CTL_CMDMOTIONSTOP:																						//停止
					
				if(motor_en_flag_temp != motor_en_flag_temp_old){	
					nwrite = DISABLE_MOTOR;
					motor_ctl(SET_DESIRED_STATE,&nwrite,MotorPort);	
				}			
					
				break;
				
				case CTL_CMDMOTIONSTART:																					//开始工作
				
				if(motor_en_flag_temp_old == CTL_CMDMOTIONSTOP){
					nwrite = ENABLE_POSITION_MODE;
					motor_ctl(SET_DESIRED_STATE,&nwrite,MotorPort);					
				}
				pthread_mutex_lock(&mutex_gait_msg);					//获取电机运动状态，socket
				state_temp = state_now;
				pthread_mutex_unlock(&mutex_gait_msg);
				
				switch(state_now){
					case 01:																//预紧点，不能是单个位置点，需要在合适的步态和力矩下开始运动
					if(state_temp != state_old){
						motor_cmd_position = motor_para_init_temp.preload_position;
						motor_ctl(SET_MOTION,&motor_cmd_position,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
					}
					motor_speed = motor_ctl(GET_CURRENT,NULL,MotorPort);
					motor_position = motor_ctl(GET_POSITION,NULL,MotorPort);
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;	

					printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
					break;
						
					case 02:																//拉扯阶段，此阶段需要快速。因此将此阶段分为两段，一段是直接快速运动，当靠近最大位置时再引入力矩环
					if(state_temp != state_old){
						motor_cmd_position = max_position;
						motor_ctl(SET_MOTION,&motor_cmd_position,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
					}
					
					motor_position = motor_ctl(GET_POSITION,NULL,MotorPort);			
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;
					
					pthread_mutex_lock(&mutex_force);			//获取当前力矩									
					force_temp = force_now;
					pthread_mutex_unlock(&mutex_force);					
						
					if(abs(motor_position.temp - motor_cmd_position) < 100){

						max_force = max_force + force_temp;			
						max_force_cnt++;		

					}										
					printf("%u motor_position = %d force_temp = %d motor_cmd_position = %d\n",time_now,motor_position.temp,force_temp,motor_cmd_position);
					break;
						
					case 03:					//回归零点，这个阶段就是快速就够了
					if(state_temp != state_old){
						nset_acc =500000;
						motor_ctl(SET_MAX_ACC,&nset_acc,MotorPort);
						nset_acc =50000;
						motor_cmd_position = motor_para_init_temp.zero_position;
						motor_ctl(SET_MOTION,&motor_cmd_position,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
						
						motor_state = motor_ctl(GET_MOTOR_FUALT,NULL,MotorPort);						
      			pthread_mutex_lock(&mutex_run_info);			
     		 		motor_module_run_info.motor_driver_state = motor_state.temp;		//获取驱动器的状态，每个步态周期一次，0为全部正常，512为过流报警
     			 	pthread_mutex_unlock(&mutex_run_info);  				
						
					}
					motor_speed = motor_ctl(GET_ACTUAL_SPEED,NULL,MotorPort);
					motor_position = motor_ctl(GET_POSITION,NULL,MotorPort);
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;	
					printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
					break;
					default:
						usleep(10000);
					break;
				}
					
				if((state_temp == 1)&&(state_old == 3)&&(max_force_cnt != 0)){
					
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;	
					
					max_force = (uint32_t)(max_force/max_force_cnt);
					deltav_force = motor_para_init_temp.max_force - max_force;
					max_position = max_position - deltav_force*2;
					
					if(max_position < motor_para_init_temp.max_position - 4000){
						max_position = motor_para_init_temp.max_position - 4000;
					}else if(max_position > motor_para_init_temp.max_position + 4000){
						max_position = motor_para_init_temp.max_position + 4000;
					}
					
					max_force = 0;
					max_force_cnt = 0;
				}
				state_old = state_temp;
				break;
				default:
					usleep(100000);
				break;
			}
			motor_en_flag_temp_old = motor_en_flag_temp;
		}
	}	
	close(MotorPort);	
}


char* zeromq_msg_getdata(char* msg,char *type,uint8_t len)
{
	char *p,*s;
    	char temp[256];

	p = strstr(msg,type);
	memset(temp,0,sizeof(temp));
	s = temp;
	
	if(p != NULL){
		p = p + len;
		while((*p != 0x20)&&(*p != 0)){
			if((uint16_t*)p < (uint16_t*)msg + 1024){
				*s = *p;
				s++;
				p++;
			}else{
				return NULL;
			}
		}
		s =temp;
		return s;
	}
	return NULL;
}


//socket通讯线程，处理获取到的数据。
void thread_gait_zeromq(void)
{
	uint32_t state_temp;
	
	void *context = zmq_ctx_new ();
	void *requester = zmq_socket (context, ZMQ_SUB);
	zmq_connect (requester, "tcp://localhost:8011");
	zmq_setsockopt(requester, ZMQ_SUBSCRIBE, "", 0);
	int request_nbr;
	char* s;

	while (1) {
		char buffer [1024],temp[32];
		memset(buffer,0,sizeof(buffer));
		zmq_recv (requester, buffer, sizeof(buffer), 0);

		s = zeromq_msg_getdata(buffer,GAIT,sizeof(GAIT));
		if(s != NULL){
			
			if(*s == 0x41)				//"B"
				state_temp = 1;
			else if(*s == 0x42)		//"C"
				state_temp = 2;
			else if(*s == 0x43)		//"A"
				state_temp = 3;
				
			pthread_mutex_lock(&mutex_gait_msg);
			state_now = state_temp;
			pthread_mutex_unlock(&mutex_gait_msg);	
				
		}
//		s = zeromq_msg_getdata(buffer,FORCE,sizeof(FORCE));
//		if(s != NULL){
//			sprintf(temp,"%s",s);
//			printf("Received state is %s\n",temp);
	}
	zmq_close (requester);
	zmq_ctx_destroy (context);
	return ; 
}



//与系统管理模块通讯
int thread_client_zeromq(void)
{
	//  Socket to talk to clients
	void *context = zmq_ctx_new ();
	void *responder = zmq_socket (context, ZMQ_REP);
	int rc = zmq_bind (responder, "tcp://*:8000");
	assert (rc == 0);


	CTLCmdMsgTypeDef cmdmotormsg;
	memset(&cmdmotormsg,0,sizeof(cmdmotormsg));

	CTLCmdRtMsgTypeDef sendrdymsg;
	memset(&sendrdymsg,0,sizeof(sendrdymsg));
	sendrdymsg.nodeID = NODEID_OF_MOTOR;

/* 1.电机启动，准备就绪，等待接受控制模块初始化指令*/
/*2. 接收控制模块指令，并执行，返回结果*/
/*3. 根据分辨 cmdmotormsg.opt 参数进行处理*/
/*4. 完成后返回结果给 ctrl node*/

	sleep(1);
	sem_post(&sem_motor);
	

	while (1) {
		/* 1.电机启动，准备就绪，等待接受控制模块初始化指令*/
		int nbytes = zmq_recv (responder, &cmdmotormsg, sizeof(cmdmotormsg), 0);
		assert (nbytes != -1);
		if(cmdmotormsg.nodeID == NODEID_OF_MOTOR){
		
      switch(cmdmotormsg.opt){
      	case CTL_CMDINITIAL:
      		
      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDINITIAL;
      	pthread_mutex_unlock(&mutex_client_msg);
      	
      	sem_post(&sem_motor);
      	sem_wait(&sem_client);     
      	 	    
      	sendrdymsg.rt = CTL_READY;   
      	   	  		
      	break;
      	
      	case CTL_CMDMOTIONSTART:
      	
      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDMOTIONSTART;
      	pthread_mutex_unlock(&mutex_client_msg);
      	
      	break	;
      	
      	case CTL_CMDPOWERDOWN:

      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDPOWERDOWN;
      	pthread_mutex_unlock(&mutex_client_msg);      	
      	
      	break;
      	
      	case CTL_CMDMOTIONSTOP:
      		
      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDMOTIONSTOP;
      	pthread_mutex_unlock(&mutex_client_msg);      	
      	
      	break;
      	
      	case CTL_CMDMOTIONSLEEP:
      	
      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDMOTIONSLEEP;
      	pthread_mutex_unlock(&mutex_client_msg);  
      	
      	break;
      	
      	default:
      	
      	break; 
      }
      

      zmq_send (responder, &sendrdymsg, sizeof(sendrdymsg), 0);
      
      
      
      
      
                
//			if(cmdmotormsg.opt == CTL_CMDSTART){
//
//				pthread_mutex_lock(&mutex_client_msg);			//获取当前使能状态									
//				motor_en_flag = 1;
//				pthread_mutex_unlock(&mutex_client_msg);	
//                    
//				/*2. 接收控制模块指令，并执行，返回结果*/
//				sendrdymsg.rt = CTL_READY;
//				if(zmq_send (responder, &sendrdymsg, sizeof(sendrdymsg), 0) < 0){
//					return -1;
//				}else{
//
//				}
//			}
		}
	}
	zmq_close (responder);
	zmq_ctx_destroy (context);	        
  return 0;	
}




void thread_motor_module_run_info(void)
{
	struct motor_module_run_info_t motor_module_run_info_temp;
	uint32_t force_temp;
	float pot_temp;
	
	while(1){
		
		pthread_mutex_lock(&mutex_gait_msg);
		motor_module_run_info_temp = motor_module_run_info;
		pthread_mutex_unlock(&mutex_gait_msg);	
		
		pthread_mutex_lock(&mutex_force);			//获取当前力矩									
		force_temp = force_now;
		pthread_mutex_unlock(&mutex_force);			
		
		pthread_mutex_lock(&mutex_pot);			//获取当前力矩									
		pot_temp = pot_now;
		pthread_mutex_unlock(&mutex_pot);		
		
		usleep(100000);
	}	
		
}