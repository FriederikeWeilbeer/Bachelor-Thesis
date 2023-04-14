# import required packages
import time
import sys
import numpy as np
import zmq
import multiprocessing
from thymiodirect import Connection
from thymiodirect import Thymio

port = 5556

ROBOT_SPEED = 350
TURN_SPEED = 200
DISTANCE = -50


# set up zmq
def setUpZMQ():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else "5556"

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://192.168.188.62:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower


def calculate_point(xl, yl, oxl, oyl):
    # calculate the point to follow
    x = xl + DISTANCE * oxl
    y = yl + DISTANCE * oyl
    print(f'point to follow: {x}, {y}')
    return x, y


def go_to_point(ox, oy, xf, yf, x, y):
    # vector to destination
    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    #angle_radians = np.arctan2([oy, dy], [ox, dx])
    #angle = [np.rad2deg(angle_radians[0]), np.rad2deg(angle_radians[1])]

    angle = [np.rad2deg(np.arctan2([oy, dy], [ox, dx])[0]),
             np.rad2deg(np.arctan2([oy, dy], [ox, dx])[1])]

    # turn left when point on the left side of the robot
    if angle[0] - angle[1] > 5:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle[0] - angle[1] < -5:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    # go straight when point in front of the robot
    elif abs(dx) > 5 or abs(dy) > 5:
        set_robot_speed(robot, ROBOT_SPEED, ROBOT_SPEED)

    else:
        stop_robot(robot)


# Robot controller
def stop_robot(robot):
    """Set both wheel robot_speeds to 0 to stop the robot"""
    robot['motor.left.target'] = 0
    robot['motor.right.target'] = 0


def set_robot_speed(robot, left_robot_speed, right_robot_speed):
    """Set both wheel robot_speeds to the given values"""
    robot['motor.left.target'] = left_robot_speed
    robot['motor.right.target'] = right_robot_speed


def main(use_sim=False, ip='192.168.188.62', port=0):
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

        # initialize variables
        robot_state = 'off'

        print('ready')

        # Main loop
        while True:

            # Receive and handle the message from the ZMQ server
            try:
                message = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
                topic = message[0]
                # print(message)
                if topic == '42':
                    robot_state = message[1]
                    print(robot_state)
                    if robot_state == 'quit':
                        stop_robot(robot)
                        break
                    if robot_state == 'off':
                        stop_robot(robot)
                # handle message for follower
                elif topic == '2':
                    if robot_state == 'on':
                        leader_x, leader_y, leader_orientation_x, leader_orientation_y, follower_x, follower_y, follower_orientation_x, follower_orientation_y = map(
                            float, message[1:])
                        point = calculate_point(leader_x, leader_y, leader_orientation_x, leader_orientation_y)
                        go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y, *point)
                else:
                    # Handle default message
                    stop_robot(robot)
                    pass
            except zmq.Again:
                pass
            except (ValueError, TypeError, IndexError) as e:
                print(f"Error: {e}")
                pass

    except (IndexError, ConnectionError) as err:
        if isinstance(err, IndexError):
            pass
        elif isinstance(err, ConnectionError):
            print("Connection Error")
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
    processes = [multiprocessing.Process(target=main, args=(False, "192.168.188.62", port,))]

    # start processes
    for p in processes:
        p.start()

    # wait for processes to finish
    for p in processes:
        p.join()
