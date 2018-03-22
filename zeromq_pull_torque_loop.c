//zeromq_pull_fixed_position.c
#include "shiki.h"

#define VELOCITY_MODE_MAX_SPEED 1500000
#define VELOCITY_MODE_MAX_ACC 2000000
#define SELF_CHECK_POT_VALUE 0.3
#define SELF_CHECK_FORCE_VALUE 200
														
#define PULL_FIX_POSITION 0
#define PULL_FORCE_TORQUE 1

#define GAIT_B_MODE PULL_FORCE_TORQUE
														
int motor_ctl(char *msg, void *para,struct motor_ctl_t *rev,int port);

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
pthread_mutex_t mutex_info;

sem_t sem_client;
sem_t sem_motor;
sem_t sem_pot_check;
sem_t sem_force_check;
sem_t sem_self_check;

uint32_t force_now;										//串口上报的力传感器值
float pot_now;
uint32_t state_now;										//socket收到的电机运动状态命令
int32_t motor_en_flag;										//1:工作   0:不工作   通过互斥锁操作
struct motor_module_run_info_t motor_module_run_info;
struct motor_module_check_info_t motor_module_check_info;
struct motor_para_init_t motor_para_init;

//获取力传感器数据，一帧的格式为 帧头：0x53 数据：0x** 0x** 校验：两个数据的与 帧尾：0x59
int get_force(int port,uint32_t *msg)
{

	int32_t 		readcnt = 0,nread,ncheck=0,state = 0;
	uint8_t			data[256],datagram[64];
	float	  		adc_temp;	
	uint32_t 		force_temp;
	int try_t = 256;
	while(try_t--){
		datagram[0] = 0;
		nread = read(port,datagram,1);
		if(nread<0){
			printf("read error nread=%d\n",readcnt);
			return -1;	
		}
		
		if(nread == 0){					
			return -1;
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
			}else{
				data[readcnt] = datagram[0];
				if(data[3] == data[1]^data[2]){
					state=2;
					readcnt++;
					break;
				}else{
					
					state = 0;
					readcnt = 0;
					
					pthread_mutex_lock(&mutex_info);			
					motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
     			pthread_mutex_unlock(&mutex_info);					
     			
					break;
				}
			}
		case 2:
			if(datagram[0]==0x59){
				data[readcnt] = datagram[0];
//				adc_temp = ((float)((data[2]<<8)|data[1]))/1000;
//				force_temp = ((uint32_t)(((data[2]<<8)|data[1])-600))/100;

				force_temp = (uint32_t)((data[2]<<8)|data[1]);
//				if(force_temp > 100){
//					force_temp = force_temp -100;		//减100是为了将力传感器的数据最小值矫正到0
//				}
//				else{
//					force_temp = 0;
//				}
//				force_temp = (uint32_t)((data[2]<<8)|data[1]);
				state = 0;
				readcnt= 0;
				*msg = force_temp;
				return 0;
				break;
			}else{

				state = 0;
				readcnt= 0;
				
				pthread_mutex_lock(&mutex_info);			
      	motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
     		pthread_mutex_unlock(&mutex_info);	
     			
				break;
			}
		}
	}
	return -1;
}

int get_pot(int port,float *msg)
{

	int32_t 		readcnt = 0,nread,ncheck=0,state = 0;
	uint8_t			data[256],datagram[64];
	float	  		adc_temp;	
	int try_t = 256;
	while(try_t--){
		datagram[0] = 0;
		nread = read(port,datagram,1);
		
		if(nread<0){
			return -1;
		}
		
		if(nread == 0){	
			return -1;
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
			}else{
				data[readcnt] = datagram[0];
				if(data[3] == data[1]^data[2]){
					state=2;
					readcnt++;
					break;
				}else{
					state = 0;
					readcnt = 0;
					
					pthread_mutex_lock(&mutex_info);			
					motor_module_run_info.pot_senser_error++;		//校验错误，收集错误信息
					pthread_mutex_unlock(&mutex_info);
					
					break;
				}
			}
		case 2:
			if(datagram[0]==0x59){
				data[readcnt] = datagram[0];
				adc_temp = ((float)((data[2]<<8)|data[1]))/1000;					
				state = 0;
				readcnt= 0;
				*msg = adc_temp;
				return 0;
				break;
			}
			else{
				state = 0;
				readcnt= 0;
				
				pthread_mutex_lock(&mutex_info);			
      	motor_module_run_info.pot_senser_error++;		//校验错误，收集错误信息
     		pthread_mutex_unlock(&mutex_info);
     							
				break;
			}
		}
	}
	return -1;
}


//驱动电机的接口，通过串口ASCII码的格式与驱动器通讯，通讯分两种
//一种是带有参数的，需要将参数传入到para
//不带参数的para传入NULL即可
//shiki.h文件里有所需要的通讯格式的宏定义
//返回为驱动器的反馈（v ***; ok; e **;）
int motor_ctl(char *msg, void *para,struct motor_ctl_t *rev,int port)
{
	int32_t readcnt = 0,nread,nwrite;
	char com[20];
	uint8_t data[256],datagram[64];
	int *p = para;
	int try_cmd = 5;
	int try_t = 1024;
	struct motor_ctl_t temp;

	if(para == NULL){
		sprintf(com,msg,NULL);	
	}else{
		sprintf(com,msg,*p);
	}
	while(try_cmd--){
		nwrite = write(port,com,strlen(com));		//发送串口命令
	
		memset(data,'\0',256);
		while(try_t--){
			nread = read(port,datagram,1);	//获取串口命令
			if(nread<0){
				return -1;		
			}
			if(nread == 0){
				printf("Communication fail\n");
				
				pthread_mutex_lock(&mutex_info);			
				motor_module_run_info.motor_driver_error++;		//通讯失败错误信息累计
				pthread_mutex_unlock(&mutex_info); 
	     	     			 	
				return -1;
			}
			data[readcnt] = datagram[0];
			readcnt++;
			if(datagram[0]==0x0D){	//最后一位为“\n”判断接收到最后一位后再处理数据
				memcpy(temp.com,data,sizeof(data));
	//			printf("rev = %s\n",temp.com);
				if(strstr(temp.com,"v")!=0){	

					sscanf(temp.com,"%*s%d",&temp.temp);
					if((void*)rev != NULL){
						memcpy(rev,&temp,sizeof(struct motor_ctl_t));
					}
					return 0;
						
				}else if(strstr(temp.com,"e")!=0){
					sscanf(temp.com,"%*s%d",&temp.state);
					printf("motor error = %d\n",temp.state);
					
					pthread_mutex_lock(&mutex_info);			
					motor_module_run_info.motor_driver_error++;		//通讯失败错误信息累计
					pthread_mutex_unlock(&mutex_info);
	     		break;
				}else if(strstr(temp.com,"ok")!=0){
					return 0;
				}	
			}
		}
	}
	return -1;
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
	pthread_mutex_init(&mutex_info,NULL);
	
	
	

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
	int i,ret;
	uint32_t force_t,force_temp[12];
	
	FrocePort = tty_init(FORCE_PORT_NUM);
	if(FrocePort<0){
		pthread_mutex_lock(&mutex_info);
		motor_module_check_info.force_port_state = PORT_OPEN_ERROR;
		pthread_mutex_unlock(&mutex_info);
		sem_post(&sem_force_check);
	}
	
	driver_init(FrocePort,FORCE_PORT_NUM);	
	
	ret = get_force(FrocePort,&force_temp[0]);
	
	if(ret == -1){
		
		pthread_mutex_lock(&mutex_info);
		motor_module_check_info.force_port_state = NO_DATA;	
		pthread_mutex_unlock(&mutex_info);
		sem_post(&sem_force_check);
		
	}else if(ret == 0){
		
		pthread_mutex_lock(&mutex_info);
		motor_module_check_info.force_port_state = PORT_OK;	
		pthread_mutex_unlock(&mutex_info);
		sem_post(&sem_force_check);			
		
	}
	
	
	
	while(1){
		
		tcflush(FrocePort,TCIFLUSH);					//清除掉串口缓存，不然串口会缓存过多数据，导致实时性降低
		
		for(i=0;i<5;i++){	
			ret = get_force(FrocePort,&force_temp[i]);	
			if(ret == -1){
				pthread_mutex_lock(&mutex_info);			
				motor_module_run_info.force_senser_error++;		//校验错误，收集错误信息
				pthread_mutex_unlock(&mutex_info);					
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
	int i,ret;
	float pot_t,pot_temp[12];
	
	AdcPort = tty_init(ADC_PORT_NUM);

	if(AdcPort<0){
		pthread_mutex_lock(&mutex_info);
		motor_module_check_info.pot_port_state = PORT_OPEN_ERROR;
		pthread_mutex_unlock(&mutex_info);
		sem_post(&sem_pot_check);
	}	
	
	driver_init(AdcPort,ADC_PORT_NUM);	
	
	ret = get_pot(AdcPort,&pot_temp[0]);
	
	if(ret == -1){
		
		pthread_mutex_lock(&mutex_info);
		motor_module_check_info.pot_port_state = NO_DATA;	
		pthread_mutex_unlock(&mutex_info);
		sem_post(&sem_pot_check);
		
	}else if(ret == 0){
		
		pthread_mutex_lock(&mutex_info);
		motor_module_check_info.pot_port_state = PORT_OK;	
		pthread_mutex_unlock(&mutex_info);
		sem_post(&sem_pot_check);			
		
	}
		
	while(1){
		
		tcflush(AdcPort,TCIFLUSH);					//清除掉串口缓存，不然串口会缓存过多数据，导致实时性降低
		
		for(i=0;i<10;i++)
		{	
			ret = get_pot(AdcPort,&pot_temp[i]);
			if((i>0)&(i<9)){
				pot_t = pot_temp[i]+pot_t ;
				if(ret == -1){
					pthread_mutex_lock(&mutex_info);			
					motor_module_run_info.pot_senser_error++;		//校验错误，收集错误信息
					pthread_mutex_unlock(&mutex_info);	
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
	float pot_temp;
	int i,nwrite,index,pndex,nset_acc,max_force_cnt,motor_speed_t,motor_speed_t_old;
	uint32_t state_temp,state_old,gait_temp,force_temp,max_force;
	uint32_t time_now,time_mark,init_position[10];
	int32_t deltav_force,integral_force,init_force[10];
	struct timeval tv;
	struct timespec ts;
	int timeout = 1000;
	struct motor_ctl_t motor_position,motor_state,motor_speed;
	int32_t motor_en_flag_temp,motor_en_flag_temp_old;
	struct motor_para_init_t motor_para_init_temp;

	MotorPort = tty_init(MOTOR_PORT_NUM);
	driver_init(MotorPort,MOTOR_PORT_NUM);

	time_t log_time = time(NULL);   
	struct tm *log_tp = localtime(&log_time);  	
	char log_path[32];
	
	sprintf(log_path,"./motor_log/motor_log_%d_%02d_%02d_%02d%02d.txt",log_tp->tm_year + 1900,log_tp->tm_mon+1,log_tp->tm_mday,log_tp->tm_hour,log_tp->tm_min);
	FILE *log_fp = fopen(log_path,"w");
	
	
	while(1){

	//		printf("enter force\n");
	//		fgets(s,20,stdin);	
	//		sscanf(s,"%d",&force_command);
	
		sem_wait(&sem_motor);
  	pthread_mutex_lock(&mutex_client_msg);	
		motor_para_init_temp = motor_para_init;
		pthread_mutex_unlock(&mutex_client_msg);			
		max_position = motor_para_init_temp.max_position;
		deltav_motor_old = 0;			
		max_force_cnt = 0;
		
		gettimeofday(&tv,NULL);
		time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);		//获取系统时间，单位为ms


		while(1){
			
			pthread_mutex_lock(&mutex_client_msg);			//获取当前使能状态									
			motor_en_flag_temp = motor_en_flag;
			pthread_mutex_unlock(&mutex_client_msg);		
//			motor_en_flag_temp = CTL_CMDMOTIONSTART;
				
			switch(motor_en_flag_temp){			
				case CTL_CMDINITIAL:											//自检
				if(motor_en_flag_temp_old == 0){															

					gettimeofday(&tv,NULL);
					ts.tv_sec = tv.tv_sec;
					ts.tv_nsec = tv.tv_usec*1000 + timeout*1000*1000;
					ts.tv_sec += ts.tv_nsec/(1000*1000*1000);
					ts.tv_nsec %= (1000*1000*1000);

					if((sem_timedwait(&sem_pot_check,&ts) == 0)&&(sem_timedwait(&sem_force_check,&ts) == 0)){
						printf("success get senser data\n");
					}else{
						printf("motor module self check abort:can not get senser data\n");	
					}						
					
					nwrite = VELOCITY_MODE_MAX_ACC;
					motor_ctl(SET_VELOCITY_ACC,&nwrite,NULL,MotorPort);
		
					nwrite = VELOCITY_MODE_MAX_ACC;
					motor_ctl(SET_VELOCITY_DEC,&nwrite,NULL,MotorPort);
					
					nset_acc = motor_para_init_temp.nset_acc;											//驱动器的最大加速度设置参数
					motor_ctl(SET_MAX_DEC,&nset_acc,NULL,MotorPort);
					nset_acc = motor_para_init_temp.nset_acc;
					motor_ctl(SET_MAX_ACC,&nset_acc,NULL,MotorPort);					

					motor_cmd_velocity = 200000;																		//设置运动速度为14000rpm 此参数需要可以配置
					motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);	
										
					while(1){
						
						nwrite = ENABLE_POSITION_MODE;
						motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
						
						motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
						
						pthread_mutex_lock(&mutex_pot);			//互斥锁
						pot_temp = pot_now;
						pthread_mutex_unlock(&mutex_pot);	
					
						if((pot_temp>(SELF_CHECK_POT_VALUE+0.02))||(pot_temp<(SELF_CHECK_POT_VALUE-0.08))){
	
							motor_cmd_position =motor_position.temp - (SELF_CHECK_POT_VALUE - pot_temp)*10000;
							motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
							motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);				
							printf("what is pot_temp: %f\n",pot_temp);
						
						}else{
							motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);	
							nwrite = 35000;
							motor_ctl(SET_POSITION,&nwrite,NULL,MotorPort);
							printf("what is pot_temp: %f\n",pot_temp);
							break;
						}
					
					}
					
					motor_cmd_velocity = 100000;	
					motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);	

					while(1){

						int32_t init_force_temp[10];
						uint32_t init_position_temp[10],init_position_overrange;
						
						motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
						
						memcpy(init_force_temp,init_force+1,sizeof(init_force)-4);
						printf("init_temp = ");
						for(i=0;i<10;i++){
							printf("%d ",init_force_temp[i]);	
						}
						printf("\n");						
						memcpy(init_force,init_force_temp,sizeof(init_force)-4);
						
						pthread_mutex_lock(&mutex_force);			//互斥锁
						force_temp = force_now;
						pthread_mutex_unlock(&mutex_force);	
																	
						init_force[9] = SELF_CHECK_FORCE_VALUE - force_temp;
						printf("init_force = ");
						for(i=0;i<10;i++){
							printf("%d ",init_force[i]);	
						}
						printf("\n");
						
						
						memcpy(init_position_temp,init_position+1,sizeof(init_position)-4);
						memcpy(init_position,init_position_temp,sizeof(init_position)-4);												
						init_position[9] = motor_position.temp;					
						printf("init_position = ");
						for(i=0;i<10;i++){
							printf("%d ",init_position[i]);	
						}
						printf("\n");
						
						int init_ret;
						init_ret = 1;
						for(i=0;i<10;i++){
							if((abs(init_force[i]) - 25) < 0){
								init_ret = init_ret&1;
							}else{
								init_ret = init_ret&0;
							}
						}
						
						if(!init_ret){
														
							motor_cmd_position =motor_position.temp - init_force[9]*100;
							printf("what is motor_cmd_position = %d\n",motor_cmd_position);
							motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
							motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
							printf("what is init_force = %d\n",init_force[9]);	
													
						}else{
							motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
							for(i=0;i<10;i++){
								init_position_overrange = init_position_overrange + init_position[i];
							}
							motor_para_init_temp.preload_position = init_position_overrange/10;
							printf("what is preload_position = %d\n",motor_para_init_temp.preload_position);
							break;					
						}					
					}

					motor_cmd_velocity = motor_para_init_temp.max_velocity;																		//设置运动速度为14000rpm 此参数需要可以配置
					motor_ctl(SET_VELOCITY,&motor_cmd_velocity,NULL,MotorPort);					

					motor_cmd_position = motor_para_init_temp.zero_position;
//					motor_cmd_position = motor_para_init_temp.preload_position;
					motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
					motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);			
																	
					sem_post(&sem_client);
					printf("this is a test start\n");						
				}
				
				break;
				
				case CTL_CMDPOWERDOWN:																				//关机
					
				nwrite = DISABLE_MOTOR;
				motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
				sem_post(&sem_client);
				return;
					
				break;
							
				case CTL_CMDMOTIONSLEEP:																		//停机
					
				if(motor_en_flag_temp != motor_en_flag_temp_old){	
					
					nwrite = ENABLE_POSITION_MODE;
					motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);					
					
					motor_cmd_position = motor_para_init_temp.zero_position;
					motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
					motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);				
					sem_post(&sem_client);
				}
					
				break;
						
				case CTL_CMDMOTIONSTOP:																						//停止
					
				if(motor_en_flag_temp != motor_en_flag_temp_old){	
					
					
					nwrite = DISABLE_MOTOR;
					motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
					sem_post(&sem_client);	
				}			
					
				break;
				
				case CTL_CMDMOTIONSTART:																					//开始工作
				if(motor_en_flag_temp != motor_en_flag_temp_old){
					nwrite = ENABLE_POSITION_MODE;
					motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
					sem_post(&sem_client);					
				}
				pthread_mutex_lock(&mutex_gait_msg);					//获取电机运动状态，socket
				state_temp = state_now;
				pthread_mutex_unlock(&mutex_gait_msg);
				
				switch(state_temp){
					case 01:																//预紧点，不能是单个位置点，需要在合适的步态和力矩下开始运动
					if(state_temp != state_old){
						
						integral_force = 0;
												
						nwrite = ENABLE_POSITION_MODE;
						motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
						
						motor_cmd_position = motor_para_init_temp.preload_position;
						motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
					}
					motor_ctl(GET_CURRENT,NULL,&motor_speed,MotorPort);
					motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;	

					pthread_mutex_lock(&mutex_force);			//获取当前力矩									
					force_temp = force_now;
					pthread_mutex_unlock(&mutex_force);	

					//printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
					printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);//time=0 force=123 position=0
					fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
					break;
						
					case 02:																//拉扯阶段，此阶段需要快速。因此将此阶段分为两段，一段是直接快速运动，当靠近最大位置时再引入力矩环
						
					#if(GAIT_B_MODE==PULL_FORCE_TORQUE)
					
					if(state_temp != state_old){
						
						motor_ctl(TRAJECTORY_ABORT,NULL,NULL,MotorPort);
						
						nwrite = ENABLE_VELOCITY_MODE;
						motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
					}
					
					motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);			
//					motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;
					
					pthread_mutex_lock(&mutex_force);			//获取当前力矩									
					force_temp = force_now;
					pthread_mutex_unlock(&mutex_force);					
					
					deltav_force = motor_para_init_temp.max_force - force_temp;
					
					if(deltav_motor > motor_para_init_temp.pid_umax){
						if(abs(deltav_force) > 200){
							index = 0;
						}else{
							index = 1;
							if(deltav_force < 0){
								integral_force = integral_force + deltav_force;
							}
						}
					}else if(deltav_motor < motor_para_init_temp.pid_umin){
						if(abs(deltav_force) > 200){
							index = 0;
						}else{
							index = 1;
							if(deltav_force > 0){
								integral_force = integral_force + deltav_force;
							}
						}
					}else{
						if(abs(deltav_force) > 200){
							index = 0;
						}else{
							index = 1;
							integral_force = integral_force + deltav_force;					
						}
					}

					if(abs(deltav_force) < 10){
						pndex = 0;
					}else{
						pndex = motor_para_init_temp.pid_kp;	//1000
					}					
					
					index = motor_para_init_temp.pid_ki*index;
					
					motor_speed_t = 0-(deltav_force*pndex + index*integral_force);
					
					if(motor_speed_t > VELOCITY_MODE_MAX_SPEED)
						motor_speed_t = VELOCITY_MODE_MAX_SPEED;
					if(motor_speed_t < -VELOCITY_MODE_MAX_SPEED)
						motor_speed_t = -VELOCITY_MODE_MAX_SPEED;
						
					if(motor_position.temp < motor_para_init_temp.max_position - 4000 ){
						if(motor_speed_t  < 0){
							motor_speed_t = 0;
						}
					}
					
					motor_ctl(SET_VELOCITY_MODE_SPEED,&motor_speed_t,NULL,MotorPort);						

					printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);//time=0 force=123 position=0
					fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,motor_speed_t);
					break;
					
					#endif // (GAIT_B_MODE==PULL_FORCE_TORQUE)
					
					#if(GAIT_B_MODE==PULL_FIX_POSITION)
					
					if(state_temp != state_old){
						motor_cmd_position = max_position;
						motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
					}
					
					motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);			
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
					printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);										
					fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
					break;					
					
					#endif // (GAIT_B_MODE==PULL_FIX_POSITION)
					
					case 03:					//回归零点，这个阶段就是快速就够了
					if(state_temp != state_old){
						
						integral_force = 0;
						
						nwrite = ENABLE_POSITION_MODE;
						motor_ctl(SET_DESIRED_STATE,&nwrite,NULL,MotorPort);
						
						motor_cmd_position = motor_para_init_temp.zero_position;
						motor_ctl(SET_MOTION,&motor_cmd_position,NULL,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,NULL,MotorPort);
						
						motor_ctl(GET_MOTOR_FUALT,NULL,&motor_state,MotorPort);						
      			pthread_mutex_lock(&mutex_info);			
     		 		motor_module_run_info.motor_driver_state = motor_state.temp;		//获取驱动器的状态，每个步态周期一次，0为全部正常，512为过流报警
     			 	pthread_mutex_unlock(&mutex_info);  				
						
					}

					pthread_mutex_lock(&mutex_force);			//获取当前力矩									
					force_temp = force_now;
					pthread_mutex_unlock(&mutex_force);	
					
					motor_ctl(GET_ACTUAL_SPEED,NULL,&motor_speed,MotorPort);
					motor_ctl(GET_POSITION,NULL,&motor_position,MotorPort);
					gettimeofday(&tv,NULL);
					time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
					time_now = (time_now - time_mark)&0x000fffff;	
					//printf("%u  motor_position = %d motor_speed = %d\n",time_now,motor_position.temp,motor_speed.temp);
					printf("time=%u force=%d position=%d\n",time_now,force_temp,motor_position.temp);//time=0 force=123 position=0
					fprintf(log_fp,"time=%u force=%d position=%d state=%d speed_cmd=%d\n",time_now,force_temp,motor_position.temp,state_temp,0);
					break;
					default:
						usleep(10000);
					break;
				}
				
				#if(GAIT_B_MODE==PULL_FIX_POSITION)	
				
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
				
				#endif // (GAIT_B_MODE==PULL_FIX_POSITION)
				state_old = state_temp;
				break;
				default:
					usleep(100000);
				break;
			}
			motor_en_flag_temp_old = motor_en_flag_temp;
		}
	}
	fclose(log_fp);	
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
//	zmq_connect (requester, "tcp://192.168.1.14:8011");
//	zmq_connect (requester, "tcp://192.168.1.11:8011");
	zmq_setsockopt(requester, ZMQ_SUBSCRIBE, "", 0);
	int request_nbr;
	char* s;

	while (1) {
		char buffer[1024],temp[32];
		memset(buffer,0,sizeof(buffer));
		zmq_recv (requester, buffer, sizeof(buffer), 0);
//		printf("what is rev : %s\n",buffer);
		s = zeromq_msg_getdata(buffer,GAIT,sizeof(GAIT));
		if(s != NULL){
		
			if(*s == 0x41)				//"B"
				state_temp = 1;
			else if(*s == 0x42)		//"C"
				state_temp = 2;
			else if(*s == 0x43)		//"A"
				state_temp = 3;
//			else if(*s == 0x4E)
//				state_temp = 3;
				
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

int get_default_settings(void)
{
	int ret;  
	void *handle;  
	const char *filepath = "./motor_para_defaults.txt";
	struct motor_para_init_t motor_para_init_temp;
	
	ret = init(filepath, &handle); 
	if(ret != 0){
		return -1;	
	}

	char valuebuf[128];
	
	ret = getValue(handle, "max_force", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.max_force);
	}

	ret = getValue(handle, "max_position", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.max_position);
	}

	ret = getValue(handle, "zero_position", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.zero_position);
	}
	
	ret = getValue(handle, "preload_position", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.preload_position);
	}
	
	ret = getValue(handle, "max_velocity", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.max_velocity);
	}

	ret = getValue(handle, "nset_acc", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.nset_acc);
	}
	
	ret = getValue(handle, "max_pot", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%f",&motor_para_init_temp.max_pot);
	}
	
	ret = getValue(handle, "pid_kp", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%f",&motor_para_init_temp.pid_kp);
	}
	
	ret = getValue(handle, "pid_ki", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%f",&motor_para_init_temp.pid_ki);
	}

	ret = getValue(handle, "pid_umax", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.pid_umax);
	}
	
	ret = getValue(handle, "pid_umin", valuebuf);  
	if (ret != 0) {  
		return -1;  
	}else{
		sscanf(valuebuf,"%d",&motor_para_init_temp.pid_umin);
	}
	
	motor_para_init = motor_para_init_temp;
	
	ret = release(&handle);  
	if (ret != 0) {  
		return -1;
	}  
      
	return 0;
}


//与系统管理模块通讯
int thread_client_zeromq(void)
{
	
	
	CTLCmdMsgTypeDef cmdmotormsg;
	memset(&cmdmotormsg,0,sizeof(cmdmotormsg));

	CTLCmdRtMsgTypeDef sendrdymsg;
	memset(&sendrdymsg,0,sizeof(sendrdymsg));
	sendrdymsg.nodeID = NODEID_OF_MOTOR;
	
	
	//  Socket to talk to clients
	void *context = zmq_ctx_new ();
	void *responder = zmq_socket (context, ZMQ_REP);
	int rc = zmq_bind (responder,"tcp://*:8000");
//	printf("what is rc %d\n",rc);
	assert (rc == 0);




//	zmq_setsockopt(responder, ZMQ_SUBSCRIBE, "", 0);
/* 1.电机启动，准备就绪，等待接受控制模块初始化指令*/
/*2. 接收控制模块指令，并执行，返回结果*/
/*3. 根据分辨 cmdmotormsg.opt 参数进行处理*/
/*4. 完成后返回结果给 ctrl node*/


  pthread_mutex_lock(&mutex_client_msg);	
	get_default_settings();	
  motor_en_flag = CTL_CMDINITIAL;
  pthread_mutex_unlock(&mutex_client_msg);
  
	sem_post(&sem_motor);
	sem_wait(&sem_client); 
	
	sleep(2);
	
	pthread_mutex_lock(&mutex_client_msg);
	motor_en_flag = CTL_CMDMOTIONSTART;
	pthread_mutex_unlock(&mutex_client_msg);
	
	
	while (1) {
		/* 1.电机启动，准备就绪，等待接受控制模块初始化指令*/
		printf("this is a test client\n");	
		zmq_recv (responder,&cmdmotormsg,sizeof(cmdmotormsg),0);
//		printf("what is rev : %d  %d \n",cmdmotormsg.nodeID,cmdmotormsg.opt);
		
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
      	sem_wait(&sem_client);
      	
      	break	;
      	
      	case CTL_CMDPOWERDOWN:

      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDPOWERDOWN;
      	pthread_mutex_unlock(&mutex_client_msg);      	
      	sem_wait(&sem_client);
      	
      	break;
      	
      	case CTL_CMDMOTIONSTOP:
      		
      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDMOTIONSTOP;
      	pthread_mutex_unlock(&mutex_client_msg);      	
      	sem_wait(&sem_client);
      	
      	break;
      	
      	case CTL_CMDMOTIONSLEEP:
      	
      	pthread_mutex_lock(&mutex_client_msg);			
      	motor_en_flag = CTL_CMDMOTIONSLEEP;
      	pthread_mutex_unlock(&mutex_client_msg);  
      	sem_wait(&sem_client);
      	
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