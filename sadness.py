# import required packages
import time
import numpy as np
import zmq
from thymiodirect import Connection
from thymiodirect import Thymio
from collections import deque

port_follower = 36965
# ip_addr = 'localhost'
ip_addr = '192.168.188.62'
simulation = False

TURN_SPEED = 90
ROBOT_SPEED = 150
CATCH_UP_SPEED = 200


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


def calculate_sadness_trajectory(start_point, end_point, num_points, point_deque):
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

    freq = (segment_length / (2.5 * np.pi))
    amp = 2

    # calculate trajectory points
    for t in t_values:
        displacement = segment_length * t
        perpendicular_displacement = amp * np.sin(freq * np.pi * t)

        x, y = start_point + displacement * normalized_direction + perpendicular_displacement * perpendicular_direction
        point_deque.append((x, y))

    # print(point_deque)
    return point_deque


def catch_up(ox, oy, xf, yf, x, y):
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
    if angle_degrees > 15:
        set_robot_speed(robot, -TURN_SPEED, TURN_SPEED)

    # turn right when point on the right side of the robot
    elif angle_degrees < -15:
        set_robot_speed(robot, TURN_SPEED, -TURN_SPEED)

    else:
        set_robot_speed(robot, CATCH_UP_SPEED, CATCH_UP_SPEED)
        return


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
        set_robot_speed(robot, 20, ROBOT_SPEED)

    # turn right when point on the right side of the robot
    elif angle_degrees < -15 and len(points) > 0:
        set_robot_speed(robot, ROBOT_SPEED, 20)

    # go straight when point in front of the robot
    elif abs(dx) > 15 or abs(dy) > 15 and len(points) > 0:
        set_robot_speed(robot, ROBOT_SPEED, ROBOT_SPEED)

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

                    trajectory_start_point = np.array([follower_x + 10 * follower_orientation_x, follower_y + 10 * follower_orientation_y])
                    trajectory_end_point = np.array([leader_x - 30 * leader_orientation_x, leader_y - 30 * leader_orientation_y])

                    # catch up when distance gets too big
                    if distance < 30:
                        stop_robot(robot)

                    elif distance > 200:
                        catch_up(follower_orientation_x, follower_orientation_y, follower_x, follower_y, trajectory_end_point[0], trajectory_end_point[1])

                    # check if the point deque is empty
                    elif len(points) == 0 and distance < 200:
                        # calculate frequency and amplitude of the happiness trajectory
                        points = calculate_sadness_trajectory(trajectory_start_point, trajectory_end_point, 6, points)

                    elif len(points) > 0 and distance < 200:
                        points = follow_trajectory(follower_orientation_x, follower_orientation_y, follower_x,
                                                   follower_y, points)

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
