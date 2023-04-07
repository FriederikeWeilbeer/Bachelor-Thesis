# import required packages
import time
import sys
import zmq
import multiprocessing
from thymiodirect import Connection
from thymiodirect import Thymio

port = 41303


# set up zmq
def setUpZMQ():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else "5556"

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://localhost:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '1')  # follower


# Robot controller
def stop_robot(robot):
    """Set both wheel robot_speeds to 0 to stop the robot"""
    robot['motor.left.target'] = 0
    robot['motor.right.target'] = 0


def set_robot_speed(robot, left_robot_speed, right_robot_speed):
    """Set both wheel robot_speeds to the given values"""
    robot['motor.left.target'] = left_robot_speed
    robot['motor.right.target'] = right_robot_speed


def getPoint(x, y):
    return True, [float(x), float(y)]


def calculateRobotCenter(data):
    x = (float(data[2]) + float(data[4]) + float(data[6]) + float(data[8])) / 4
    y = (float(data[3]) + float(data[5]) + float(data[7]) + float(data[9])) / 4
    return [x, y]


def makeVectors(data):
    vec1 = [float(data[2]), float(data[3])]
    vec2 = [float(data[4]), float(data[5])]
    vec3 = [float(data[6]), float(data[7])]
    vec4 = [float(data[8]), float(data[9])]
    return vec1, vec2, vec3, vec4


def main(use_sim=False, ip='localhost', port=0):
    """main loop of the program"""
    try:
        # Configure Interface to Thymio robot
        if use_sim:
            th = Thymio(use_tcp=True, host=ip, tcp_port=port,
                        on_connect=lambda node_id: print(f' Thymio {node_id} is connected'))
        else:
            port = Connection.serial_default_port()
            th = Thymio(serial_port=port,
                        on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))

        # Robot Connection setup
        global robot
        th.connect()  # Connect to the robot
        robot = th[th.first_node()]  # Create an object to control the robot
        print(f"{robot} connected")  # Print the robot name
        time.sleep(5)  # Delay to allow robot initialization of all variables

        # ZMQ setup
        setUpZMQ()

        print('ready')

        # Main loop
        while True:

            # Receive and handle the message from the ZMQ server
            try:
                leader_center, leader_orientation, follower_center, follower_orientation = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()

            except zmq.Again as error:
                pass

    except ConnectionError:
        print("Connection Error")
    except Exception as err:
        # Stop robot
        stop_robot(robot)
        print(err)
    except KeyboardInterrupt:
        # Stop robot
        stop_robot(robot)
        print('Keyboard Interrupt')


if __name__ == '__main__':
    print("Starting processes...")

    # spawn process for each robot
    processes = []
    # for port in ports:
    processes.append(multiprocessing.Process(target=main, args=(True, "localhost", port,)))

    # start processes
    for p in processes:
        p.start()

    # wait for processes to finish
    for p in processes:
        p.join()
