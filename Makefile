all:zeromq_pull_torque_loop.o port.o Properties.o
	gcc zeromq_pull_torque_loop.o port.o Properties.o -o zeromq_pull_torque_loop -lzmq -lpthread
zeromq_pull_torque_loop:zeromq_pull_torque_loop.c shiki.h
	gcc -c zeromq_pull_torque_loop.c
port:port.c shiki.h
	gcc -c port.c
Properties:Properties.c shiki.h
	gcc -c Properties.c
clean:
	rm -f *.o zeromq_pull_torque_loop
