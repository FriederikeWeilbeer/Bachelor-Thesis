# import required packages
import time
import sys
import zmq
import multiprocessing
from thymiodirect import Connection
from thymiodirect import Thymio

port = 35031


# set up zmq
def setUpZMQ():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else "5556"

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://localhost:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '1')  # leader


# Robot controller
def stop_robot(robot):
    """Set both wheel robot_speeds to 0 to stop the robot"""
    robot['motor.left.target'] = 0
    robot['motor.right.target'] = 0


def set_robot_speed(robot, left_robot_speed, right_robot_speed):
    """Set both wheel robot_speeds to the given values"""
    robot['motor.left.target'] = left_robot_speed
    robot['motor.right.target'] = right_robot_speed


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
        print(str(th.first_node()))

        # ZMQ setup
        setUpZMQ()

        # Initialize variables
        robot_state = 'off'  # State of the robot (on/off)
        robot_action = 'stop'  # Action of the robot
        robot_action_cur = ''  # Current action of the robot
        robot_speed = 350  # Max speed of the robot

        print('ready')

        # Main loop
        while True:

            # Receive and handle the message from the ZMQ server
            try:
                topic, data = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()

                if topic == '42':  # Handle the message for all robots
                    robot_state = data
                elif topic == str(th.first_node()):  # Handle the message for this robot
                    robot_action = data

            except zmq.Again:
                pass

            # Handle the robot state and set the action
            if robot_state == 'quit':
                stop_robot(robot)
            if robot_state == 'off':
                robot_action = 'stop'

            # Create a dictionary to map actions to function calls
            action_map = {
                'straight': lambda: set_robot_speed(robot, robot_speed, robot_speed),
                'back': lambda: set_robot_speed(robot, -robot_speed, -robot_speed),
                'stop': lambda: stop_robot(robot),
                'left': lambda: set_robot_speed(robot, int(robot_speed / 4), robot_speed),
                'right': lambda: set_robot_speed(robot, robot_speed, int(robot_speed / 4)),
                'spotleft': lambda: set_robot_speed(robot, -robot_speed, robot_speed),
                'spotright': lambda: set_robot_speed(robot, robot_speed, -robot_speed),
            }

            # Handle the robot action
            if robot_action != robot_action_cur:
                robot_action_cur = robot_action
                action_map.get(robot_action, lambda: None)()

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
    processes = [multiprocessing.Process(target=main, args=(True, "localhost", port,))]

    # start processes
    for p in processes:
        p.start()

    # wait for processes to finish
    for p in processes:
        p.join()
