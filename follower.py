# import required packages
import time
import numpy as np
import zmq
from thymiodirect import Connection
from thymiodirect import Thymio
from collections import deque

port_follower = 45735
ip_addr = 'localhost'
# ip_addr = '192.168.188.62'
simulation = True

ROBOT_SPEED = 200
TURN_SPEED = 100
DISTANCE = -100


# set up zmq
def setUpZMQ():
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    # socket.connect(f"tcp://192.168.188.62:{port}")
    socket.connect(f"tcp://{ip_addr}:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower


def calculate_point(xl, yl, oxl, oyl, point_deque):
    # calculate the point to follow
    x = xl + DISTANCE * oxl
    y = yl + DISTANCE * oyl
    point_deque.append((x, y))
    return point_deque


def go_to_point(ox, oy, xf, yf, point_deque):
    # pop first point from deque
    if len(point_deque) > 0:
        x, y = point_deque.popleft()
    else:
        return

    # vector to destination
    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    angle_radians = np.arctan2([oy, dy], [ox, dx])
    angle = [np.rad2deg(angle_radians[0]), np.rad2deg(angle_radians[1])]

    # turn left when point on the left side of the robot
    if angle[0] - angle[1] > 15:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle[0] - angle[1] < -15:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    # go straight when point in front of the robot
    elif abs(dx) > 15 or abs(dy) > 15:
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


def main(sim, ip, port):
    """main loop of the program"""
    try:
        # Robot Connection setup
        if sim:
            th = Thymio(use_tcp=True, host=ip, tcp_port=port,
                        on_connect=lambda node_id: print(f' Thymio {node_id} is connected'))
        else:
            port = Connection.serial_default_port()
            th = Thymio(serial_port=port,
                        on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))
        global robot
        th.connect()  # Connect to the robot
        robot = th[th.first_node()]  # Create an object to control the robot
        time.sleep(5)  # Delay to allow robot initialization of all variables

        # ZMQ setup
        setUpZMQ()

        # initialize variables
        robot_state = 'off'

        # initialize deque for points
        points = deque()

        print('ready')

        # Main loop
        while True:
            time.sleep(0.1)
            # Receive and handle the message from the ZMQ server
            try:
                message = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
                topic = message[0]
                if topic == '42':
                    robot_state = message[1]
                    print(robot_state)
                    if robot_state == 'quit':
                        stop_robot(robot)
                    if robot_state == 'off':
                        stop_robot(robot)
                # handle message for follower
                elif topic == '2':
                    if robot_state == 'on':
                        leader_x, leader_y, leader_orientation_x, leader_orientation_y, follower_x, follower_y, follower_orientation_x, follower_orientation_y = map(
                            float, message[1:])
                        points = calculate_point(leader_x, leader_y, leader_orientation_x, leader_orientation_y, points)
                        print('deque: ', points)
                        go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y, points)
                        # print('leader: ', leader_x, leader_y)
                        # print('follower: ', follower_x, follower_y)
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
    print('Starting follower ...')
    main(sim=simulation, ip=ip_addr, port=port_follower)
