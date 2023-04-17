# import required packages
import time
import sys
import zmq
import multiprocessing
from thymiodirect import Connection
from thymiodirect import Thymio


# set up zmq
def setUpZMQ(queue):
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://192.168.188.62:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '1')  # leader

    while True:
        # Receive and handle the message from the ZMQ server
        try:
            topic, data = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
            queue.put((topic, data))
        except zmq.Again:
            pass


# Robot controller
def stop_robot(robot):
    """Set both wheel robot_speeds to 0 to stop the robot"""
    robot['motor.left.target'] = 0
    robot['motor.right.target'] = 0


def set_robot_speed(robot, left_robot_speed, right_robot_speed):
    """Set both wheel robot_speeds to the given values"""
    robot['motor.left.target'] = left_robot_speed
    robot['motor.right.target'] = right_robot_speed


def main():
    """main loop of the program"""
    try:
        port = Connection.serial_default_port()
        th = Thymio(serial_port=port,
                    on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
        # Robot Connection setup
        th.connect()  # Connect to the robot
        robot = th[th.first_node()]  # Create an object to control the robot
        print(f"{robot} connected")  # Print the robot name
        time.sleep(5)  # Delay to allow robot initialization of all variables
        print(str(th.first_node()))

        # Initialize variables
        robot_state = 'off'  # State of the robot (on/off)
        robot_action = 'stop'  # Action of the robot
        robot_action_cur = ''  # Current action of the robot
        robot_speed = 350  # Max speed of the robot
        new_command_received = False  # Flag to indicate a new command
        action_map = {
            'straight': lambda: set_robot_speed(robot, robot_speed, robot_speed),
            'back': lambda: set_robot_speed(robot, -robot_speed, -robot_speed),
            'stop': lambda: stop_robot(robot),
            'left': lambda: set_robot_speed(robot, int(robot_speed / 3), robot_speed),
            'right': lambda: set_robot_speed(robot, robot_speed, int(robot_speed / 3)),
            'spotleft': lambda: set_robot_speed(robot, -robot_speed, robot_speed),
            'spotright': lambda: set_robot_speed(robot, robot_speed, -robot_speed),
        }

        # start ZMQ handling in a separate process
        message_queue = multiprocessing.Queue()
        zmq_process = multiprocessing.Process(target=setUpZMQ, args=(message_queue,))
        zmq_process.start()

        print('ready')

        # Main loop
        while True:

            # handle messages from ZMQ
            while not message_queue.empty():
                topic, data = message_queue.get()
                # print(topic, data)

                if topic == '42':  # Handle the message for all robots
                    robot_state = data
                elif topic == str(th.first_node()):  # Handle the message for this robot
                    robot_action = data
                else:
                    stop_robot(robot)

            # Handle the robot state and set the action
            if robot_state == 'quit':
                stop_robot(robot)
            if robot_state == 'off':
                robot_action = 'stop'

            if robot_action_cur != robot_action:
                new_command_received = True
                robot_action_cur = robot_action

            # Handle the robot action if a new command has been received
            if new_command_received:
                new_command_received = False  # Reset the flag

                # Execute the appropriate action for the current command
                action_map.get(robot_action, lambda: None)()
                robot_action_cur = robot_action
                time.sleep(0.1)

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
    main()
