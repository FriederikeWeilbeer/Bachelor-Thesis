
from pickle import FALSE
import zmq
import platform
import numpy as np
from thymiodirect import Connection
from thymiodirect import Thymio

# set up zmq subscriber, subscribing to messages with their id and with 42 as header
def setUpZMQ():
    global socket
    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect ("tcp://192.168.188.57:%s" % port)
    socket.subscribe("42")
    socket.subscribe(platform.node().replace("CP", ""))


def connectToThymio():
    global robot
    try:
        # Configure Interface to Thymio robot

        port = Connection.serial_default_port()
        th = Thymio(serial_port=port, 
                    on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
        print('Thymio connected ...')
        # Connect to Robot
        th.connect()
        robot = th[th.first_node()]
    except Exception as err:
        # Stop robot
        robot['motor.left.target'] = 0
        robot['motor.right.target'] = 0 
        print(err)

# convert string message to float value (position of the robot)
def makeStringIntoVektor(data):
    vec1 = [float(data[1]), float(data[2])]
    vec2 = [float(data[3]), float(data[4])]
    vec3 = [float(data[5]), float(data[6])]
    vec4 = [float(data[7]), float(data[8])]

    return vec1, vec2, vec3, vec4

# convert string message to float value (point to drive to)
def getPoint(data):
    return True, [float(data[2]),float(data[3])]


def goToPoint(vec1, vec4, point): 
    t =[vec1[0]-vec4[0],vec1[1]-vec4[1]]
    d =[point[0]-vec4[0],point[1]-vec4[1]]
    print(d)
    angle = np.rad2deg(np.arctan2([t[1],d[1]], [t[0], d[0]])[0]) - np.rad2deg(np.arctan2([t[1],d[1]], [t[0], d[0]])[1])
    
    if angle < -180:
        angle = 360 + angle
    if angle > 180:
        angle = 360 - angle
    print(angle)
    leftVelocity = int(200 - angle) 
    rightVelocity = int(200 + angle) 
    if abs(d[0]) <= 5 and  abs(d[1]) <= 5:
        robot['motor.left.target']  = 0
        robot['motor.right.target'] = 0
        return False
    elif abs(d[0]) < 25 and  abs(d[1]) < 25:
        if angle> 5:
            robot['motor.left.target']  = -50
            robot['motor.right.target'] =  50
            return True
        elif  angle< -5:
            robot['motor.left.target']  =   50
            robot['motor.right.target'] =  -50
            return True
        else:
            robot['motor.left.target']  = 50
            robot['motor.right.target'] = 50
            return True
    else:
        robot['motor.left.target']  = leftVelocity
        robot['motor.right.target'] = rightVelocity
        return True

def main():
    connectToThymio()
    setUpZMQ()
    while True:
        try:
            #check for a message, this will not block
            string = socket.recv_string()
            # Split the message into data and topic
            data = string.split()
            
            if data[0] == "14":
                if data[1] == "go":
                     print("go")
                     move, point = getPoint(data)
                else:
                    vec1, vec2, vec3, vec4 = makeStringIntoVektor(data)
            elif data[1] == "quit":
                robot['motor.left.target']=0
                robot['motor.right.target']=0
                break
            
            if move:
                print("hh")
                move = goToPoint(vec1, vec4 ,point)

        except Exception as err:
            None
    print("bye")




if __name__ == "__main__":
    main()