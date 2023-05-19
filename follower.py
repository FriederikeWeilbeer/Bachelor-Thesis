# import required packages
import time
import numpy as np
import zmq
from queue import Queue
from queue import PriorityQueue
from thymiodirect import Connection
from thymiodirect import Thymio
from threading import Thread
from matplotlib import pyplot as plt

port_follower = 37649
ip_addr = 'localhost'
# ip_addr = '192.168.188.62'
simulation = True

ROBOT_SPEED = 200
TURN_SPEED = 100
DISTANCE = -100


# thread to handle zmq messages
def zmq_handler(queue, point_queue):
    global save_points
    port = 5556

    # Socket to talk to server
    global socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://{ip_addr}:{port}")

    socket.setsockopt_string(zmq.SUBSCRIBE, '42')  # all robots
    socket.setsockopt_string(zmq.SUBSCRIBE, '2')  # follower

    while True:
        # print(save_points)
        # Receive and handle the message from the ZMQ server
        try:
            data = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
            topic = data.pop(0)
            if topic == '42':
                # Set a higher priority for strings starting with 42
                queue.put((1, data))
                # Empty the point queue
                point_queue.queue.clear()
                save_points = False  # Disable saving priority 2 messages
            elif save_points:
                queue.put((2, data))
        except zmq.Again:
            pass
        # print('queue size: ', point_queue.qsize())
        if point_queue.qsize() < 4:
            # Set the flag to save priority 2 messages
            save_points = True


def happiness(leader_pos, leader_orientation, follower_pos, point_queue):
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
        x, y = mid_point[0], mid_point[1] + sine_displacement * sign * DISTANCE

        # add the final point to the deque
        point_queue.put((x, y))
        sign = -sign

    # plot the points with matplotlib
    # plt.plot(*zip(*point_deque))
    # plt.show()
    return point_queue


def calculate_points(leader_pos, leader_orientation, follower_pos, point_queue):
    # calculate distance between leader and follower
    segment_length = np.linalg.norm(follower_pos - leader_pos)
    
    # calculate 5 points between leader and follower
    for i in range(5):
        # calculate the current position along the line segment
        segment_start = leader_pos - leader_orientation * (i * segment_length / 5)
        segment_end = leader_pos - leader_orientation * ((i + 1) * segment_length / 5)

        # calculate the mid-point between the start and end of the current segment
        mid_point = (segment_start + segment_end) / 2

        # calculate the final position of the mid-point, displaced by the sine wave
        x, y = mid_point[0], mid_point[1]

        # add the final point to the deque
        point_queue.put((x, y))
    print('queue', list(point_queue.queue))
    return point_queue


def go_to_point(ox, oy, xf, yf, point_queue):
    while not point_queue.empty():
        x, y = point_queue.get()
        # Vector to destination
        dx = x - xf
        dy = y - yf

        # Calculate the angle between the two vectors
        angle_radians = np.arctan2([oy, dy], [ox, dx])
        angle = [np.rad2deg(angle_radians[0]), np.rad2deg(angle_radians[1])]

        # Turn left when point on the left side of the robot
        if angle[0] - angle[1] > 15:
            set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

        # Turn right when point on the right side of the robot
        elif angle[0] - angle[1] < -15:
            set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

        # Go straight when point in front of the robot
        elif abs(dx) > 15 or abs(dy) > 15:
            set_robot_speed(robot, ROBOT_SPEED, ROBOT_SPEED)
        else:
            # Stop the robot
            stop_robot(robot)
            break  # Exit the loop when the point is reached

    return point_queue


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

        # initialize empty queue for points
        point_queue = Queue()

        # initialize follower information
        follower_orientation_x = None
        follower_orientation_y = None
        follower_x = None
        follower_y = None

        # flag to save points of priority 2 on demand
        save_points = False

        # ZMQ setup
        message_queue = PriorityQueue()
        zmq_thread = Thread(target=zmq_handler, args=(message_queue, point_queue))
        zmq_thread.start()

        # movement thread
        movement_thread = Thread(target=go_to_point, args=(follower_orientation_x, follower_orientation_y, follower_x, follower_y, point_queue,))
        movement_thread.start()

        # initialize variables
        robot_state = 'off'

        print('ready')

        # Main loop
        while True:
            # Receive and handle the message from the ZMQ server
            while not message_queue.empty():
                message = message_queue.get()
                data = message[1]
                print('data: ', data)
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

                    point_queue = calculate_points(leader_pos, leader_orientation, follower_pos, point_queue)

                    # point_queue = happiness(leader_pos, leader_orientation, follower_pos, point_queue)
                    # go to the next point in the queue
                    go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y, point_queue)
                    # while len(points) > 0:
                    #     go_to_point(follower_orientation_x, follower_orientation_y, follower_x, follower_y,
                    #                 points)
                    # print('point_queue: ', points)
                    # print('leader: ', leader_x, leader_y)
                    # print('follower: ', follower_x, follower_y)

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
