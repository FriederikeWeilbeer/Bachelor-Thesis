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

port_follower = 44131
ip_addr = 'localhost'
# ip_addr = '192.168.188.62'
simulation = True

ROBOT_SPEED = 200
TURN_SPEED = 100
DISTANCE = -100


# thread to handle zmq messages
def zmq_handler(queue):
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
            topic = data.pop(0)
            if topic == '42':
                # set a higher priority for strings starting with 42
                queue.put((1, data))
            else:
                queue.put((2, data))
        except zmq.Again:
            pass


def happiness(leader_pos, leader_orientation, follower_pos, point_deque):
    # calculate the length of the line segment between the starting and end points
    segment_length = np.linalg.norm(follower_pos - leader_pos)

    # split the segment into six smaller segments
    segment_count = 5
    segment_size = segment_length / segment_count
    sign = 1

    for i in range(segment_count):
        # calculate the current position along the line segment
        segment_start = leader_pos - leader_orientation * (i * segment_size)
        segment_end = leader_pos - leader_orientation * ((i + 1) * segment_size)

        # calculate the mid-point between the start and end of the current segment
        mid_point = (segment_start + segment_end) / 2

        # calculate the sine wave displacement at the mid-point
        sine_displacement = np.sin((np.pi / segment_count) * i)

        # calculate the final position of the mid-point, displaced by the sine wave
        final_point = mid_point[0], mid_point[1] + sine_displacement * sign * DISTANCE

        # add the final point to the deque
        point_deque.append(final_point)
        sign = -sign

    # plot the points with matplotlib
    # plt.plot(*zip(*point_deque))
    # plt.show()
    return point_deque


def calculate_sine_trajectory(start_point, end_point, num_points, point_deque):
    x_follower, y_follower = start_point
    x_leader, y_leader = end_point

    x_values = np.linspace(x_follower, x_leader, num_points)
    print('x ', x_values)
    y_values = (y_leader - y_follower) * np.sin(np.linspace(0, 2 * np.pi, num_points)) + y_follower
    print('y ', y_values)

    points = zip(x_values, y_values)
    for point in points:
        point_deque.append(point)
    # plot the points with matplotlib
    #plt.plot(*zip(*point_deque))
    #plt.show()
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


def go_to_point(ox, oy, xf, yf, points):
    # get the first point in the deque
    x, y = points[0]

    print(f'x: {x}, y: {y}')
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

        # when point reached, remove it from the deque
    if abs(dx) < 20 and abs(dy) < 20:
        points.popleft()

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

        # ZMQ setup
        message_queue = PriorityQueue()
        zmq_thread = Thread(target=zmq_handler, args=(message_queue,))
        zmq_thread.start()

        # initialize deque for points
        points = deque()

        # initialize variables
        robot_state = 'off'

        print('ready')

        # Main loop
        while True:
            # Receive and handle the message from the ZMQ server
            while not message_queue.empty():
                message = message_queue.get()
                # topic = data.pop(0)
                data = message[1]
                # print('data: ', data)
                if data[0] == 'stop':
                    robot_state = 'stop'
                    stop_robot(robot)
                elif data[0] == 'on':
                    robot_state = 'on'
                if message[0] != 1 and (robot_state == 'on' or robot_state == 'stop'):
                    leader_x, leader_y, leader_orientation_x, leader_orientation_y, follower_x, follower_y, follower_orientation_x, follower_orientation_y = map(
                        float, data)
                    follower_orientation = np.array([follower_orientation_x, follower_orientation_y])
                    follower_pos = np.array([follower_x, follower_y])
                    leader_pos = np.array([leader_x, leader_y])
                    leader_orientation = np.array([leader_orientation_x, leader_orientation_y])

                    # check if the point deque is empty
                    if len(points) == 0:
                        # points = calculate_points(leader_pos, leader_orientation, follower_pos, points)
                        points = calculate_sine_trajectory(follower_pos, leader_pos, 5, points)
                    print('points: ', points)

                    # go to the next point in the deque
                    if len(points) > 0:
                        points = go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y, points)

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
