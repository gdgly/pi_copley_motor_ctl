    //c_unix.c  
    #include <stdio.h>  
    #include <unistd.h>
    #include <sys/types.h>  
    #include <sys/socket.h>  
    #include <sys/un.h>  
    #define MOTOR_DOMAIN "/tmp/motor.domain"  
    int main(void)  
    {  
        int connect_fd;  
        int ret,cnt0,cnt1;  
        char snd_buf[1024];  
        int i;  
        static struct sockaddr_un srv_addr;  
    //creat unix socket  
        connect_fd=socket(PF_UNIX,SOCK_STREAM,0);  
        if(connect_fd<0)  
        {  
            perror("cannot create communication socket");  
            return 1;  
        }     
        srv_addr.sun_family=AF_UNIX;  
        strcpy(srv_addr.sun_path,MOTOR_DOMAIN);  
    //connect server  
        ret=connect(connect_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));  
        if(ret==-1)  
        {  
            perror("cannot connect to the server");  
            close(connect_fd);  
            return 1;  
        }  
    //send info server  
        while(1){ 
	    memset(snd_buf,0,1024);  
	    if(cnt0 < 37){
	   	 cnt1 = 1;
	    }else if(cnt0 < 70){
	    	cnt1 = 2;
	    }else if(cnt0 < 100){
	    	cnt1 = 3;
	    }else{
		cnt0 = 0;
	    }
            sprintf(snd_buf,"%d",cnt1);
            write(connect_fd,snd_buf,sizeof(snd_buf));  
	    usleep(10000);
	    cnt0++;
	}
        close(connect_fd);  
        return 0;  
    }  
