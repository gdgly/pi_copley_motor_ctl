import sys
import os
import zmq

class SYSTEM_NODE_PUB(object):
    def __init__(self):
        #bind to system stream
        self.system_node_stream = "tcp://192.168.1.12:8000"
        self.system_node_context = zmq.Context()
        self.system_node_socket = self.system_node_context.socket(zmq.REQ)
        self.system_node_socket.connect(self.system_node_stream)

        while True:
            print ("test system node cmd:")
            print ("1:cmdmotorintial")
            print ("2:cmdmotorstart")
            print ("3:cmdmotorpause")
            print ("4:cmdmotorstop")
            print ("5:cmdmotorshutdown")
            print ("6:cmdmotormode:gait")
            print ("7:cmdmotormode:fixed")
            print ("8:cmdmotormode:relax")

            self.input = input("input:")
            self.cmd = ""
            if self.input == "1":
                self.cmd = "cmdmotorintial"
            elif self.input == "2":
                self.cmd = "cmdmotorstart"
            elif self.input == "3":
                self.cmd = "cmdmotorpause"
            elif self.input == "4":
                self.cmd = "cmdmotorstop"
            elif self.input == "5":
                self.cmd = "cmdmotorshutdown"
            elif self.input == "6":
                self.cmd = "cmdmotormode:gait"
            elif self.input == "7":
                self.cmd = "cmdmotormode:fixed"
            elif self.input == "8":
                self.cmd = "cmdmotormode:relax"
            else :
                print ("illegal cmd\n")

            print ("send system :" + self.cmd)
            self.system_node_socket.send_string(self.cmd)
            self.evt = self.system_node_socket.recv()

if __name__ == "__main__":
    try:
        SYSTEM_NODE_PUB()
    except KeyboardInterrupt :
        print ("\nCtrl+C entered.\nExiting...")



