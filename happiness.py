# import required packages
import time
import numpy as np
import zmq
from queue import PriorityQueue
from thymiodirect import Connection
from thymiodirect import Thymio
from collections import deque
from threading import Thread
from matplotlib import pyplot as plt

port_follower = 33901
ip_addr = 'localhost'
# ip_addr = '192.168.188.62'
simulation = True

ROBOT_SPEED = 200
TURN_SPEED = 100
DISTANCE = -100


# thread to handle zmq messages
def zmq_handler(queue, point_deque):
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{ip_addr}:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower

    while True:
        # Receive and handle the message from the ZMQ server
        try:
            data = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
            # print('data: ', data)
            topic = data.pop(0)
            if topic == '42':
                # set a higher priority for strings starting with 42
                queue.put((1, data))
                if data[0] == 'stop':
                    stop_robot(robot)
                point_deque.clear()
            else:
                queue.put((2, data))
        except zmq.Again:
            pass


def calculate_sine_trajectory(start_point, end_point, num_points, point_deque):
    start_point = np.array(start_point)
    end_point = np.array(end_point)

    # segment direction and length
    segment_direction = end_point - start_point
    segment_length = np.linalg.norm(segment_direction)

    # normalize segment direction
    normalized_direction = segment_direction / segment_length

    # perpendicular direction to the segment
    perpendicular_direction = np.array([-normalized_direction[1], normalized_direction[0]])

    # parameter values along the trajectory
    t_values = np.linspace(0, 1, num_points)

    # calculate trajectory points
    for t in t_values:
        displacement = segment_length * t
        perpendicular_displacement = (segment_length / np.pi) * np.sin(2 * np.pi * t)

        x, y = start_point + displacement * normalized_direction + perpendicular_displacement * perpendicular_direction
        point_deque.append((x, y))
    '''
    print("point_deque", point_deque)

    # Plot the points
    plt.plot(*zip(*point_deque))
    plt.gca().invert_yaxis()  # Invert the y-axis
    plt.show()'''
    return point_deque


def calculate_points(leader_pos, leader_orientation, follower_pos, point_deque):
    # calculate distance between leader and follower
    segment_length = np.linalg.norm(follower_pos - leader_pos)

    # calculate 5 points between leader and follower
    for i in range(10):
        # calculate the current position along the line segment
        segment_start = leader_pos - leader_orientation * (i * segment_length / 5)
        segment_end = leader_pos - leader_orientation * ((i + 1) * segment_length / 5)

        # calculate the mid-point between the start and end of the current segment
        mid_point = (segment_start + segment_end) / 2

        # calculate the final position of the mid-point, displaced by the sine wave
        x, y = mid_point[0], mid_point[1]

        # add the final point to the deque
        point_deque.append((x, y))
    return point_deque


def catch_up(ox, oy, xf, yf, x, y):
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

    if abs(dx) < 100 and abs(dy) < 100:
        return

    else:
        # stop the robot
        stop_robot(robot)
        return


def follow_trajectory(ox, oy, xf, yf, points):
    # get the first point in the deque
    x, y = points[0]

    print(f'x: {x}, y: {y}')
    # vector to destination
    dx = x - xf
    dy = y - yf

    # calculate the angle between the two vectors
    angle_radians = np.arctan2([oy, dy], [ox, dx])
    angle = [np.rad2deg(angle_radians[0]), np.rad2deg(angle_radians[1])]

    if len(points) == 0:
        stop_robot(robot)
        return points

    # turn left when point on the left side of the robot
    elif angle[0] - angle[1] > 15 and len(points) > 0:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle[0] - angle[1] < -15 and len(points) > 0:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    # go straight when point in front of the robot
    elif abs(dx) > 15 or abs(dy) > 15 and len(points) > 0:
        set_robot_speed(robot, ROBOT_SPEED, ROBOT_SPEED)

    # when point reached, remove it from the deque
    if abs(dx) < 20 and abs(dy) < 20 and len(points) > 0:
        points.popleft()
        if len(points) > 0:
            x, y = points[0]

    else:
        # stop the robot
        stop_robot(robot)

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
    time.sleep(0.1)


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
        message_queue = PriorityQueue()
        zmq_thread = Thread(target=zmq_handler, args=(message_queue, points))
        zmq_thread.start()

        # initialize variables
        robot_state = 'off'

        print('ready')

        # Main loop
        while True:
            # Receive and handle the message from the ZMQ server
            while not message_queue.empty():
                message = message_queue.get()
                data = message[1]
                # print('data: ', data)
                if data[0] == 'stop':
                    stop_robot(robot)
                    points.clear()
                elif data[0] == 'on':
                    robot_state = 'on'
                elif data[0] == 'off':
                    robot_state = 'off'
                elif message[0] != 1 and robot_state == 'on':
                    leader_x, leader_y, leader_orientation_x, leader_orientation_y, follower_x, follower_y, follower_orientation_x, follower_orientation_y = map(
                        float, data)

                    # distance between the two robots
                    dx = leader_x - follower_x
                    dy = leader_y - follower_y
                    distance = np.sqrt(dx ** 2 + dy ** 2)
                    print('distance: ', distance)

                    follower_orientation = np.array([follower_orientation_x, follower_orientation_y])
                    follower_pos = np.array([follower_x, follower_y])
                    leader_pos = np.array([leader_x, leader_y])
                    leader_orientation = np.array([leader_orientation_x, leader_orientation_y])

                    # catch up when distance gets too big
                    if distance > 200:
                        catch_up(follower_orientation_x, follower_orientation_y, follower_x, follower_y, leader_x,
                                 leader_y)

                    # check if the point deque is empty
                    elif len(points) == 0 and distance < 200:
                        # points = calculate_points(leader_pos, leader_orientation, follower_pos, points)
                        points = calculate_sine_trajectory(follower_pos, leader_pos, 6, points)

                    elif len(points) > 0 and distance < 200:
                        points = follow_trajectory(follower_orientation_x, follower_orientation_y, follower_x,
                                                   follower_y, points)

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
