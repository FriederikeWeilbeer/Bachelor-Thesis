# import required packages
import time
import numpy as np
import zmq
import random
from thymiodirect import Connection
from thymiodirect import Thymio
from collections import deque

port_follower = 36965
# ip_addr = 'localhost'
ip_addr = '192.168.188.62'
simulation = False

ROBOT_SPEED = 350
CATCH_UP_SPEED = 450
SLOW_TURN = 60
TURN_SPEED = 150
SPEED1 = 100
SPEED2 = 400


# thread to handle zmq messages
def setup_zmq():
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{ip_addr}:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower


def jostle(ox, oy, xf, yf, x, y):
    # vector to destination
    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    angle_radians = np.arctan2(oy, ox) - np.arctan2(dy, dx)
    angle_degrees = np.rad2deg(angle_radians)

    # Normalize the angle between -180 and 180 degrees
    angle_degrees = (angle_degrees + 180) % 360 - 180
    # print('angle', angle_degrees)

    # turn left when point on the left side of the robot
    if angle_degrees > 10:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle_degrees < -10:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    else:
        set_robot_speed(robot, ROBOT_SPEED, ROBOT_SPEED)
        return


def tailgate(ox, oy, xf, yf, x, y):
    # vector to destination
    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    angle_radians = np.arctan2(oy, ox) - np.arctan2(dy, dx)
    angle_degrees = np.rad2deg(angle_radians)

    # Normalize the angle between -180 and 180 degrees
    angle_degrees = (angle_degrees + 180) % 360 - 180
    # print('angle', angle_degrees)

    # turn left when point on the left side of the robot
    if angle_degrees > 10:
        set_robot_speed(robot, -SLOW_TURN, SLOW_TURN)

    # turn right when point on the right side of the robot
    elif angle_degrees < -10:
        set_robot_speed(robot, SLOW_TURN, -SLOW_TURN)

    else:
        set_robot_speed(robot, CATCH_UP_SPEED, CATCH_UP_SPEED)
        return


def push(ox, oy, xf, yf, x, y, speed):
    # vector to destination
    dx = x - xf
    dy = y - yf

    # alternate between SPEED1 and SPEED2
    if speed == SPEED1:
        speed = SPEED2
    else:
        speed = SPEED1

    # calculate the angle between the two vectors
    angle_radians = np.arctan2(oy, ox) - np.arctan2(dy, dx)
    angle_degrees = np.rad2deg(angle_radians)

    # Normalize the angle between -180 and 180 degrees
    angle_degrees = (angle_degrees + 180) % 360 - 180
    # print('angle', angle_degrees)

    # turn left when point on the left side of the robot
    if angle_degrees > 15:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle_degrees < -15:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    else:
        set_robot_speed(robot, speed, speed)
        return speed

    # default return value
    return speed


def follow_trajectory(ox, oy, xf, yf, points):
    # get the first point in the deque
    x, y = points[0]

    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    angle_radians = np.arctan2(oy, ox) - np.arctan2(dy, dx)
    angle_degrees = np.rad2deg(angle_radians)

    # Normalize the angle between -180 and 180 degrees
    angle_degrees = (angle_degrees + 180) % 360 - 180
    # print('angle: ', angle_degrees)

    if len(points) == 0:
        stop_robot(robot)
        return points

    # turn left when point on the left side of the robot
    if angle_degrees > 15 and len(points) > 0:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle_degrees < -15 and len(points) > 0:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    # go straight when point in front of the robot
    elif abs(dx) > 15 or abs(dy) > 15 and len(points) > 0:
        set_robot_speed(robot, CATCH_UP_SPEED, CATCH_UP_SPEED)

    # when point reached, remove it from the deque
    if abs(dx) < 15 and abs(dy) < 15 and len(points) > 0:
        points.popleft()
        if len(points) > 0:
            x, y = points[0]

    # return the updated deque of points
    return points


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

        # initialize deque for points
        points = deque()

        # ZMQ setup
        setup_zmq()
        # initialize variables
        robot_state = 'off'
        count = 0
        mode = 1
        speed = SPEED1

        print('ready')

        # Main loop
        while True:
            time.sleep(0.1)
            # Receive and handle the message from the ZMQ server
            try:
                message = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
                topic = message[0]
                # print('message: ', message)
                # messages for all robots
                if topic == '42':
                    points.clear()
                    if message[1] == 'quit':
                        robot_state = 'off'
                        stop_robot(robot)
                        break
                    elif message[1] == 'on':
                        robot_state = 'on'
                    elif message[1] == 'stop':
                        stop_robot(robot)

                # messages for this particular robot
                elif topic == '2' and robot_state == 'on':
                    leader_x, leader_y, leader_orientation_x, leader_orientation_y, follower_x, follower_y, follower_orientation_x, follower_orientation_y = map(
                        float, message[1:])

                    # distance between the two robots
                    dx = leader_x - follower_x
                    dy = leader_y - follower_y
                    distance = np.sqrt(dx ** 2 + dy ** 2)

                    trajectory_end_point = np.array(
                        [leader_x - 40 * leader_orientation_x, leader_y - 40 * leader_orientation_y])

                    if count > 40:
                        mode = random.randint(1, 3)
                        print(mode)
                        count = 0
                        points.clear()

                    # stop when distance gets too small
                    if distance < 35:
                        stop_robot(robot)

                    if mode == 1:
                        tailgate(follower_orientation_x, follower_orientation_y, follower_x, follower_y,
                                 trajectory_end_point[0], trajectory_end_point[1])
                        count += 1

                    if mode == 2:
                        jostle(follower_orientation_x, follower_orientation_y, follower_x, follower_y,
                               trajectory_end_point[0], trajectory_end_point[1])
                        count += 1

                    if mode == 3:
                        speed = push(follower_orientation_x, follower_orientation_y, follower_x,
                                     follower_y,
                                     trajectory_end_point[0], trajectory_end_point[1], speed)

                    count += 1

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
