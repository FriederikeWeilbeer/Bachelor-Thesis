# import required packages
import time
import numpy as np
import zmq
from thymiodirect import Connection
from thymiodirect import Thymio

port_follower = 42827
ip_addr = 'localhost'
simulation = True

ROBOT_SPEED = 350
TURN_SPEED = 200
DISTANCE = -100

MOTION = 'straight'         # 'straight' or 'happiness'


# set up zmq
def setUpZMQ():
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    # socket.connect(f"tcp://192.168.188.62:{port}")
    socket.connect(f"tcp://localhost:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower


def calculate_point(xl, yl, oxl, oyl):
    # calculate the point to follow
    x = xl + DISTANCE * oxl
    y = yl + DISTANCE * oyl
    return x, y


def go_to_point(ox, oy, xf, yf, x, y):
    # vector to destination
    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    angle_radians = np.arctan2([oy, dy], [ox, dx])
    angle = [np.rad2deg(angle_radians[0]), np.rad2deg(angle_radians[1])]

    if angle[0] - angle[1] > 15:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle[0] - angle[1] < -15:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    # no follower motion selected
    if MOTION == 'straight':
        straight_motion(dx, dy)

    # happiness motion selected
    elif MOTION == 'happiness':
        happiness_motion(dx, dy, ox, oy, xf, yf, x, y)

    else:
        stop_robot(robot)


def happiness_motion(dx, dy, ox, oy, xf, yf, x, y):
    # calculate the distance to the destination
    distance = np.sqrt(dx ** 2 + dy ** 2)

    # sine wave between follower and destination



def straight_motion(dx, dy):
    # go straight when point in front of the robot
    if abs(dx) > 15 or abs(dy) > 15:
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
                        amplitude = 50
                        frequency = 0.5
                        go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y, *point, amplitude, frequency)

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
