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
    socket.connect("tcp://192.168.188.62:%s" % port)
    socket.subscribe("14")
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
    return True, [float(data[2]), float(data[3])]


# robot will turn to the point then drive straight; will do correction when needed
def goToPoint(vec1, vec4, point):
    orientation = [vec1[0] - vec4[0], vec1[1] - vec4[
        1]]  # vector of the orientation; vector from bottom left corner to top left corner od the aruco marker
    destination = [point[0] - vec4[0], point[1] - vec4[
        1]]  # vector to destiation; vector from bottom left corner of aruco marker to point choosen

    # angle between (0,1) and orientation or destination respectively 
    angle = [np.rad2deg(np.arctan2([orientation[1], destination[1]], [orientation[0], destination[0]])[0]),
             np.rad2deg(np.arctan2([orientation[1], destination[1]], [orientation[0], destination[0]])[1])]

    # turn left when point on the left side of the robot
    if angle[0] - angle[1] > 5:
        robot['motor.left.target'] = -100
        robot['motor.right.target'] = 100
        return True
    # turn right when point on the left side of the robot
    elif angle[0] - angle[1] < -5:
        robot['motor.left.target'] = 100
        robot['motor.right.target'] = -100
        return True
    # move forward when not close to the point
    if abs(destination[0]) > 5 or abs(destination[1]) > 5:
        robot['motor.left.target'] = 200
        robot['motor.right.target'] = 200
        return True
    # stop
    else:
        robot['motor.left.target'] = 0
        robot['motor.right.target'] = 0
        return False


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
                if data[1] == "go":
                    print("go")
                    move, point = getPoint(data)
                else:
                    vec1, vec2, vec3, vec4 = makeStringIntoVektor(data)
            elif data[1] == "quit":
                robot['motor.left.target'] = 0
                robot['motor.right.target'] = 0
                break
            # start moving, when point was chosen 
            if move:
                move = goToPoint(vec1, vec4, point)

        except Exception as err:
            None
    print("bye")


if __name__ == "__main__":
    main()
