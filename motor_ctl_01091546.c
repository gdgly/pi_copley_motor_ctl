//torque_loop_thread.c
#include "shiki.h"

struct motor_ctl_t motor_ctl(char *msg, void *para, int port);

void thread_force_port(void);
void thread_motor_port(void);
void thread_motor_socket(void);

pthread_mutex_t mutex_force;
pthread_mutex_t mutex_socket;

uint32_t force_now;										//�����ϱ�����������ֵ
uint32_t state_now;										//socket�յ��ĵ���˶�״̬����

#define MOTOR_DOMAIN "/tmp/motor.domain"  	//socket��ַ������ģ�����ͨ����·������socket����

//���ô��ڲ����������� ��żλ��
int set_opt(int fd,int nSpeed,int nBits,char nEvent,int nStop)
{
	struct termios newtio,oldtio;
	//�������д��ڲ�������
	if(tcgetattr(fd,&oldtio)!=0) {
		perror("Setup Serial 1");
		return -1;
	}
	bzero(&newtio,sizeof(newtio));
	//���øñ�־���ſ������ò����ʣ��ַ���С������λ��ֹͣλ�Ȳ���
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//����ֹͣλ
	switch(nBits) {
		case 7:
			newtio.c_cflag |=CS7;
			break;
		case 8:
			newtio.c_cflag |=CS8;
			break;
	}
	//������żУ��λ
	switch(nEvent) {
		case 'O':	//��
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK|ISTRIP);
		break;
		case 'E':	//ż
			newtio.c_iflag |= (INPCK|ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
		break;
		case 'N':	//��У��
			newtio.c_cflag &= ~PARENB;
		break;
	}
	//���ò�����
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
	//����ֹͣλ
	if(nStop ==1) {
		newtio.c_cflag &= ~CSTOPB;
	}else if(nStop==2) {
		newtio.c_cflag |= CSTOPB;
	}
	//���õȴ�ʱ�����С�����ַ������õȴ�ʱ��Ϊ1s
	newtio.c_cc[VTIME]=1;
	newtio.c_cc[VMIN]=0;
	//����δ�����ַ�
	tcflush(fd,TCIFLUSH);
  	//����������
	if((tcsetattr(fd,TCSANOW,&newtio))!=0) {
		perror("com set error");
		return -1;
	}
//	fprintf(stderr,"set done!\n");
	return 0;
}

//���ڳ�ʼ�������ô��ڲ������������ϵ�Ĭ�ϲ�����Ϊ9600����Ҫ�����Ϊ115200
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
//		write(port,baud,strlen(baud));			//�������ϵ�Ĭ�ϲ�����Ϊ9600����Ҫ�����Ϊ115200
//
//		usleep(500000);

 		ret = set_opt(port,115200,8,'N',1);
  		if(ret < 0) {
			perror("set_opt error\n");
		}
		return;
	}
}



//�򿪴����豸����
//����������Ĭ��ΪUSB1����λ�ƴ���Ĭ��ΪUSB0��������������Ĭ��ΪUSB2
int tty_init(int CurrentPort)
{
	char com[16],pnum[2];
	int TempPort;
	//�򿪴����豸
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
  	//�ָ�����Ϊ����״̬
  	if(fcntl(TempPort,F_SETFL,0)<0)
  		printf("fcntl failed\n");
  	else
  		printf("fcntl=%d\n",fcntl(TempPort,F_SETFL,0));
  	//�����Ƿ�Ϊ�ն��豸
  	if(isatty(STDIN_FILENO)==0) {
  		printf("standard input is not a terminal device\n");
	}
  	else
  		printf("isatty success!\n");

	return TempPort;
}


//��ȡ�����������ݣ�һ֡�ĸ�ʽΪ ֡ͷ��0x53 ���ݣ�0x** 0x** У�飺�������ݵ��� ֡β��0x59
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
				if(force_temp > 100){
					force_temp = force_temp -100;		//��100��Ϊ�˽�����������������Сֵ������0
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

//��������Ľӿڣ�ͨ������ASCII��ĸ�ʽ��������ͨѶ��ͨѶ������
//һ���Ǵ��в����ģ���Ҫ���������뵽para
//����������para����NULL����
//shiki.h�ļ���������Ҫ��ͨѶ��ʽ�ĺ궨��
//����Ϊ�������ķ�����v ***; ok; e **;��
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
	nwrite = write(port,com,strlen(com));		//���ʹ�������
	
	memset(data,'\0',256);
	while(1)
	{
		nread = read(port,datagram,1);	//��ȡ��������
		if(nread<0)
		{
			printf("read error nread=%d\n",readcnt);	
		}
		if(nread == 0)
		{
			printf("hello\n");
			return temp;
		}
		data[readcnt] = datagram[0];
		readcnt++;
		if(datagram[0]==0x0D)	//���һλΪ��\n���жϽ��յ����һλ���ٴ�������
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

//���������������ݽ��д���
//ð���㷨+ƽ������ȥ��������С��һ����
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
	int motor_temp,deltav_motor,motor_temp_p,motor_temp_v,pid_umax,pid_umin;
	char s[20];	
	int i,nwrite,index;
	uint32_t force_t,force_temp[12],force_command;
	uint32_t	time_now,time_mark;
	int32_t deltav_force,integral_force;
	struct timeval tv;*/


	pthread_t tid1,tid2,tid3;		//�����������߳�
	
	pthread_mutex_init(&mutex_force,NULL);			//����������������
	pthread_mutex_init(&mutex_socket,NULL);

	pthread_create(&tid1,NULL,(void*)thread_force_port,NULL);			//�����߳�
	pthread_create(&tid2,NULL,(void*)thread_motor_port,NULL);
	pthread_create(&tid3,NULL,(void*)thread_motor_socket,NULL);
	
	pthread_join(tid1,NULL);			//�ȴ��߳̽�����ÿ���̶߳���while(1)ѭ�������Բ������
	pthread_join(tid2,NULL);
	pthread_join(tid3,NULL);
}


//����������ȡ�߳�
void thread_force_port(void)
{
	int FrocePort;			//���������Ķ˿ں� USB0
	char s[20];	
	int i,nwrite,index;
	uint32_t force_t,force_temp[12];
	
	FrocePort = tty_init(FORCE_PORT_NUM);
	driver_init(FrocePort,FORCE_PORT_NUM);	
	
	while(1){
		
		tcflush(FrocePort,TCIFLUSH);					//��������ڻ��棬��Ȼ���ڻỺ��������ݣ�����ʵʱ�Խ���
		
		for(i=0;i<5;i++){	
			force_temp[i] = get_force(FrocePort);	
		}
		force_t = bubble_sort_and_average(force_temp,5);	
		
		pthread_mutex_lock(&mutex_force);			//������
		force_now = force_t;
		pthread_mutex_unlock(&mutex_force);
	}
	
	close(FrocePort);
}


//��������߳�
void thread_motor_port(void)
{
	int MotorPort,cnt;			
	int deltav_motor,deltav_motor_old,motor_temp_p,motor_temp_v,pid_umax,pid_umin;
	char s[20];	
	int i,nwrite,index,nset_acc;
	uint32_t force_command,state_temp,state_old;
	uint32_t time_now,time_mark;
	int32_t deltav_force,integral_force;
	struct timeval tv;
	struct motor_ctl_t motor_temp;

	MotorPort = tty_init(MOTOR_PORT_NUM);
	driver_init(MotorPort,MOTOR_PORT_NUM);
	
while(1){	

//		printf("enter force\n");
//		fgets(s,20,stdin);	
//		sscanf(s,"%d",&force_command);
		force_command = 550;			//���������������أ��˲�����Ҫ��������
		integral_force = 0;				//���ػ�·�Ļ�����
		pid_umax = 500;						//�����ֱ��͵������λ
		pid_umin = -500;					//�����ֱ��͵���С��λ
		deltav_motor_old = 0;			
		
		gettimeofday(&tv,NULL);
		time_mark = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);		//��ȡϵͳʱ�䣬��λΪms

		motor_temp_v = 1400000;																		//�����˶��ٶ�Ϊ14000rpm �˲�����Ҫ��������
		motor_ctl(SET_VELOCITY,&motor_temp_v,MotorPort);					

		nset_acc = 50000;					//�������������ٶ����ò���
		motor_ctl(SET_MAX_ACC,&nset_acc,MotorPort);
		motor_ctl(SET_MAX_DEC,&nset_acc,MotorPort);

		while(1){

			pthread_mutex_lock(&mutex_socket);					//��ȡ����˶�״̬��socket
			state_temp = state_now;
			pthread_mutex_unlock(&mutex_socket);
			switch(state_now){
				case 01:																//Ԥ���㣬�����ǵ���λ�õ㣬��Ҫ�ں��ʵĲ�̬�������¿�ʼ�˶�
				if(state_temp != state_old){
					motor_temp_p = 18000;
					motor_ctl(SET_MOTION,&motor_temp_p,MotorPort);
					motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
				}
				break;
				case 02:																//�����׶Σ��˽׶���Ҫ���١���˽��˽׶η�Ϊ���Σ�һ����ֱ�ӿ����˶������������λ��ʱ���������ػ�

				motor_temp = motor_ctl(GET_POSITION,NULL,MotorPort);			
				
				gettimeofday(&tv,NULL);
				time_now = (uint32_t)(tv.tv_sec*1000+tv.tv_usec/1000);
				time_now = (time_now - time_mark)&0x000fffff;	

				if(motor_temp.temp > 1000){ 						//�������λ�ã�������ص㣩��0ʱ���������ػ��ĵ���1000��
					motor_temp_p = 0;											//Ϊ��ʹ�������ͨ������˽�λ������ֱ�����õ�������ص㡣
					integral_force = 0;
					if(nset_acc == 50000){
						nset_acc =1000000;
						motor_ctl(SET_MAX_ACC,&nset_acc,MotorPort);
						nset_acc =200000;
						motor_ctl(SET_MAX_DEC,&nset_acc,MotorPort);
					}
	
					motor_ctl(SET_MOTION,&motor_temp_p,MotorPort);
					motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
					printf("%u  motor_temp = %d\n",time_now,motor_temp.temp);
				}else{
					if(nset_acc == 1000000){							//������ٶȹ���ʱ���������ػ����ȶ���������������ػ�·ʱ��Ҫ�����ٶȽ���
						nset_acc =200000;
						motor_ctl(SET_MAX_DEC,&nset_acc,MotorPort);
						nset_acc = 50000;
						motor_ctl(SET_MAX_ACC,&nset_acc,MotorPort);
					}
	
					pthread_mutex_lock(&mutex_force);			//��ȡ��ǰ����
					deltav_force = force_command - force_now;
					printf("%u %d\n",time_now,force_now);
					pthread_mutex_unlock(&mutex_force);	
			
					if(deltav_motor > pid_umax){					//���ֿ����ͺͻ����޷�
						if(abs(deltav_force) > 200){
							index = 0;
						}else{
							index = 1;
							if(deltav_force < 0){
								integral_force = integral_force + deltav_force;
							}
						}
					}else if(deltav_motor < pid_umin){
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
					deltav_motor = deltav_force*6 + index*integral_force*0.1;		//PI���������ػ�·����ʱ��̣ܶ���200ms���ڣ�����ȶ��Բ�������Ҫ�ģ���·��Ҳ���ٱ��ֳ�����
	
					motor_temp_p = motor_temp.temp - deltav_motor;
					printf("integal = %d daltav_force = %d deltav_motor = %d motor_temp = %d\n",integral_force,deltav_force,deltav_motor,motor_temp.temp);

					if((deltav_motor_old > 1000)&&(deltav_motor > 1000)&&(abs(deltav_motor - deltav_motor_old) < 200)&&(cnt < 10)){		//Ϊ�˿���ͨ������1000������������֮���ͨѶ
						cnt++;
					}else{
						cnt = 0;
						motor_ctl(SET_MOTION,&motor_temp_p,MotorPort);
						motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
					}
					deltav_motor_old = deltav_motor;
				}
				break;
				case 03:					//�ع���㣬����׶ξ��ǿ��پ͹���
				if(state_temp != state_old){
					nset_acc =1000000;
					motor_ctl(SET_MAX_ACC,&nset_acc,MotorPort);
					nset_acc =50000;
					motor_temp_p = 25000;
					motor_ctl(SET_MOTION,&motor_temp_p,MotorPort);
					motor_ctl(TRAJECTORY_MOVE,NULL,MotorPort);
				}
				break;
				default:
					usleep(10000);
				break;
			}
			state_old = state_temp;
		}
	}	
	close(MotorPort);	
}

//socketͨѶ�̣߳�����socket���󣬴����ȡ�������ݡ�
void thread_motor_socket(void)
{

	socklen_t clt_addr_len;
	int listen_fd;
	int com_fd;
	int ret;
	int i;
	uint32_t state_temp;
	static char recv_buf[1024];
	int len;
	struct sockaddr_un clt_addr;
	struct sockaddr_un srv_addr;
	listen_fd=socket(PF_UNIX,SOCK_STREAM,0);
	if(listen_fd<0){
		perror("cannot create communication socket");
		return;
        }
          
	//set server addr_param
	srv_addr.sun_family=AF_UNIX;
	strncpy(srv_addr.sun_path,MOTOR_DOMAIN,sizeof(srv_addr.sun_path)-1);
	unlink(MOTOR_DOMAIN);
	//bind sockfd & addr
	ret=bind(listen_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
	if(ret==-1){
		perror("cannot bind server socket");
		close(listen_fd);
		unlink(MOTOR_DOMAIN);
		return;
        }
	//listen sockfd
	ret=listen(listen_fd,1);
	if(ret==-1){
		perror("cannot listen the client connect request");
		close(listen_fd);
		unlink(MOTOR_DOMAIN);
		return;
	}
	//have connect request use accept
	len=sizeof(clt_addr);
	com_fd=accept(listen_fd,(struct sockaddr*)&clt_addr,&len);
	if(com_fd<0){
		perror("cannot accept client connect request");
		close(listen_fd);
		unlink(MOTOR_DOMAIN);
		return;
	}
	while(1){
		memset(recv_buf,0,1024);
		int num=read(com_fd,recv_buf,sizeof(recv_buf));			//����Ϊ��ȡ������������������Ϊ0 ʱ˵��socket�Ͽ������¼���
		if(num > 0){
//			printf("Message from client (%d)) :%s\n",num,recv_buf);
			sscanf(recv_buf,"%d",&state_temp);									//��λ�����ݴ���
			pthread_mutex_lock(&mutex_socket);
			state_now = state_temp;
			pthread_mutex_unlock(&mutex_socket);
		}else{
			close(com_fd);
			printf("num = %d\n",num);
     	   		ret=listen(listen_fd,1);
     	   		if(ret==-1){
				perror("cannot listen the client connect request");
				close(listen_fd);
      	      			unlink(MOTOR_DOMAIN);
       	     			return;  
       			}  
        		//have connect request use accept  
			len=sizeof(clt_addr);  
			com_fd=accept(listen_fd,(struct sockaddr*)&clt_addr,&len);  
			if(com_fd<0){  
            			perror("cannot accept client connect request");  
            			close(listen_fd);  
            			unlink(MOTOR_DOMAIN);  
            			return;  
			}	    
		}  
	}  
	close(com_fd);  
	close(listen_fd);  
	unlink(MOTOR_DOMAIN);  
	return;  
}
