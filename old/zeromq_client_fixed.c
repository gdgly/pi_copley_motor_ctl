//zeromq_client_fix.c
#include "shiki.h"

int main (void)
{

	char s[20];
	//  Socket to talk to clients
	
	CTLCmdMsgTypeDef cmdmotormsg;
	memset(&cmdmotormsg,0,sizeof(cmdmotormsg));
	cmdmotormsg.nodeID = NODEID_OF_MOTOR;
	
	

	CTLCmdRtMsgTypeDef sendrdymsg;
	memset(&sendrdymsg,0,sizeof(sendrdymsg));
	
	
	
	void *context = zmq_ctx_new ();
	void *requester = zmq_socket (context, ZMQ_REQ);
	zmq_connect (requester, "tcp://localhost:8000");

//	zmq_setsockopt(responder, ZMQ_SUBSCRIBE, "", 0);
	
	
	while (1) {
	
		printf("enter command\n");
	
	
	
		fgets(s,20,stdin);
	
		int ret = sscanf(s,"%d",(int*)&cmdmotormsg.opt);
		if(ret != 0){
		
			zmq_send (requester,&cmdmotormsg, sizeof(cmdmotormsg),0);
			
			printf("is data send? \n");
					
			zmq_recv(requester,&sendrdymsg, sizeof(sendrdymsg), 0);	
		
		}
	}
	return 0;
}