//zeromq_server_fix.c
#include "shiki.h"

int main (void)
{
	int32_t cnt=0;
	//  Socket to talk to clients
	void *context = zmq_ctx_new ();
 	void *responder = zmq_socket (context, ZMQ_PUB);
	int rc = zmq_bind (responder, "tcp://*:8011");
	assert (rc == 0);
	while (1) {
		char buffer [10];
	cnt++;
	if(cnt < 27)
		zmq_send (responder, "Gait:B", 6, 0);
	else if(cnt < 60)
		zmq_send (responder, "Gait:C", 6, 0);
	else if(cnt < 100)
		zmq_send (responder, "Gait:A", 6, 0);
	else
		cnt = 0;
		
	usleep (15000);          //  Do some 'work'
//	zmq_send (responder, "Gait:N", 6, 0);
	}
	return 0;
}11111111111111