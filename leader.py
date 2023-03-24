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
    socket.connect("tcp://192.168.0.132:%s" % port)
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


# convert string message to motor values
def moveRobot(data):
    if data[1] == "left":
        robot['motor.left.target'] = -100
        robot['motor.right.target'] = 100
        return True
    elif data[1] == "right":
        robot['motor.left.target'] = 100
        robot['motor.right.target'] = -100
        return True
    elif data[1] == "stop":
        robot['motor.left.target'] = 0
        robot['motor.right.target'] = 0
        return True
    elif data[1] == "start":
        robot['motor.left.target'] = 100
        robot['motor.right.target'] = 100
        return True
    else:
        return False


# convert string message to float value (position of the robot)
def makeStringIntoVektor(data):
    vec1 = [float(data[1]), float(data[2])]
    vec2 = [float(data[3]), float(data[4])]
    vec3 = [float(data[5]), float(data[6])]
    vec4 = [float(data[7]), float(data[8])]

    return vec1, vec2, vec3, vec4


def main():
    connectToThymio()
    setUpZMQ()
    while True:
        try:
            # check for a message, this will not block
            string = socket.recv_string()
            # Split the message into data and topic
            data = string.split()

            # process data
            if data[0] == "14":
                moveRobot(data)
                if data[1] == "start":
                    print("start")
            elif data[1] == "stop":
                robot['motor.left.target'] = 0
                robot['motor.right.target'] = 0
                break

        except Exception as err:
            None
    print("bye")


if __name__ == "__main__":
    main()
