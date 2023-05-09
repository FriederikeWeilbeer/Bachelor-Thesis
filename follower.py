# import required packages
import time
import numpy as np
import zmq
from thymiodirect import Connection
from thymiodirect import Thymio
from collections import deque
import threading
from queue import Queue
from matplotlib import pyplot as plt

port_follower = 38311
ip_addr = 'localhost'
# ip_addr = '192.168.188.62'
simulation = True

ROBOT_SPEED = 200
TURN_SPEED = 100
DISTANCE = -100
message_queue = Queue()


# thread to handle zmq messages
def zmq_handler(queue):
    port = 5556
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{ip_addr}:{port}")
    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower
    while True:
        # Receive and handle the message from the ZMQ server
        try:
            message = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
            queue.put(message)
        except zmq.Again:
            pass


# get the current follower position
def get_follower_position():
    message = message_queue.get()


# set up zmq
def setUpZMQ():
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # connect to server
    socket.connect(f"tcp://{ip_addr}:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower


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


def calculate_point(xl, yl, oxl, oyl, point_deque):
    # calculate the point to follow
    x = xl + DISTANCE * oxl
    y = yl + DISTANCE * oyl
    point_deque.append((x, y))
    return point_deque


def go_to_point(ox, oy, xf, yf, point_deque):
    x, y = point_deque.popleft()
    print("dest point: ", x, y)
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
        # stop the robot
        stop_robot(robot)
        # return the updated deque of points
    return point_deque


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
        # setUpZMQ()
        zmq_thread = threading.Thread(target=zmq_handler, args=(message_queue,))
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
                        follower_orientation = np.array([follower_orientation_x, follower_orientation_y])
                        follower_pos = np.array([follower_x, follower_y])
                        leader_pos = np.array([leader_x, leader_y])
                        leader_orientation = np.array([leader_orientation_x, leader_orientation_y])

                        # check if the point deque is empty
                        #if not points:
                        points = calculate_point(leader_x, leader_y, leader_orientation_x, leader_orientation_y, points)
                            # points = happiness(leader_pos, leader_orientation, follower_pos, points)
                        # go to the next point in the deque
                        go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y, points)
                        print('leader: ', leader_x, leader_y)
                        print('follower: ', follower_x, follower_y)
                        # while len(points) > 0:
                        #     go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y,
                        #                 points)
                        # print('point_queue: ', points)
                        # print('leader: ', leader_x, leader_y)
                        # print('follower: ', follower_x, follower_y)
                else:
                    # Handle default message
                    stop_robot(robot)
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
