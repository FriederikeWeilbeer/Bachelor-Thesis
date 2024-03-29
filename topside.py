# import required packages
import time
import zmq
import pygame
import sys
import aruco_detection
import cv2
import cv2.aruco as aruco
import numpy as np

leader_id = 1
follower_id = 2
simulation_mode_enabled = False
illustration_mode_enabled = True
emotion = 'anger'
#emotion = 'happiness'
#emotion = 'sadness'


def setUpZMQ(port):
    global zmq_socket
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:%s" % port)


def getKeyboardInput():
    keys = pygame.key.get_pressed()
    topic = leader_id

    if keys[pygame.K_q]:
        topic = 42
        message = "quit"
    elif keys[pygame.K_s]:
        topic = 42
        message = "on"
    elif keys[pygame.K_UP] and keys[pygame.K_LEFT]:
        message = "left"
    elif keys[pygame.K_UP] and keys[pygame.K_RIGHT]:
        message = "right"
    elif keys[pygame.K_UP]:
        message = "straight"
    elif keys[pygame.K_LEFT] and keys[pygame.K_DOWN]:
        message = "spotleft"
    elif keys[pygame.K_RIGHT] and keys[pygame.K_DOWN]:
        message = "spotright"
    elif keys[pygame.K_DOWN]:
        message = "back"
    elif keys[pygame.K_LEFT]:
        message = "tightleft"
    elif keys[pygame.K_RIGHT]:
        message = "tightright"
    else:
        topic = 42
        message = "stop"
    return topic, message


def get_marker_info(marker_id, ids, corners):
    markers = []
    for i in range(len(ids)):
        if ids[i][0] == marker_id:
            # Thymio facing up (-y-axis)
            top_right = {"x": corners[i][0][0][0], "y": corners[i][0][0][1]}
            bottom_right = {"x": corners[i][0][1][0], "y": corners[i][0][1][1]}
            bottom_left = {"x": corners[i][0][2][0], "y": corners[i][0][2][1]}
            top_left = {"x": corners[i][0][3][0], "y": corners[i][0][3][1]}

            # calculate the center of the marker
            sum_x = np.sum(top_right["x"] + bottom_right["x"] + bottom_left["x"] + top_left["x"])
            sum_y = np.sum(top_right["y"] + bottom_right["y"] + bottom_left["y"] + top_left["y"])
            center = str(sum_x / 4) + ' ' + str(sum_y / 4)

            # calculate the vector from corner 4 to corner 1
            vector = top_right["x"] - bottom_right["x"], top_right["y"] - bottom_right["y"]

            # normalize the vector
            orientation = vector / np.linalg.norm(vector)
            orientation = str(orientation[0]) + ' ' + str(orientation[1])
            markers.append({"center": center, "orientation": orientation})

    if markers:
        center = markers[0]["center"]
        orientation = markers[0]["orientation"]
        return center, orientation
    else:
        return None, None


def findArucoMarkers(img, arucoDict, arucoParam):
    # convert the video image to gray image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    aruco.drawDetectedMarkers(img, corners)
    return corners, ids


def calculate_points(start_point, end_point, num_points):
    start_point = np.array(start_point)
    end_point = np.array(end_point)

    # segment direction and length
    segment_direction = end_point - start_point
    segment_length = np.linalg.norm(segment_direction)

    normalized_direction = segment_direction / segment_length

    t_values = np.linspace(0, 1, num_points)

    trajectory_points = []
    for t in t_values:
        displacement = segment_length * t
        x, y = start_point + displacement * normalized_direction
        trajectory_points.append((x, y))

    return trajectory_points


def calculate_trajectory_points(start_point, end_point):
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
    if emotion == 'happiness':
        num_points = 12
    elif emotion == 'anger':
        num_points = 6
    elif emotion == 'sadness':
        num_points = 12
    t_values = np.linspace(0, 1, num_points)

    # calculate trajectory points
    trajectory_points = []
    if emotion == 'happiness':
        for t in t_values:
            displacement = segment_length * t
            perpendicular_displacement = (segment_length / (1 * np.pi)) * np.sin(2 * np.pi * t)

            x, y = start_point + displacement * normalized_direction + perpendicular_displacement * perpendicular_direction
            trajectory_points.append((x, y))

    if emotion == 'anger':
        # parameter values along the trajectory
        zig_zag_distance = segment_length / (num_points - 1)
        t = 1

        # calculate trajectory points
        for i in range(2, num_points):
            # displacement = zig_zag_distance * (i % 2) * 2 - zig_zag_distance
            displacement = 25 * t

            x, y = start_point + normalized_direction * (zig_zag_distance * i) + displacement * perpendicular_direction
            trajectory_points.append((x, y))
            t *= -1
    if emotion == 'sadness':
        freq = 2
        amp = 20
        for t in t_values:
            displacement = segment_length * t
            perpendicular_displacement = amp * np.sin(freq * np.pi * t)

            x, y = start_point + displacement * normalized_direction + perpendicular_displacement * perpendicular_direction
            trajectory_points.append((x, y))
    return trajectory_points


def mark_points_on_image(img, points):
    for point in points:
        cv2.circle(img, (int(point[0]), int(point[1])), 2, (0, 255, 0), -1)


def main(screen_size=(100, 100), zmq_port=5556):
    pygame.init()
    screen = pygame.display.set_mode(screen_size)
    setUpZMQ(zmq_port)

    if not simulation_mode_enabled:

        # setup aruco Dictionary
        key = getattr(aruco, f'DICT_{4}X{4}_{100}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()

        # setup video capture
        cap = cv2.VideoCapture(0)
        # Check if the webcam is opened correctly
        if not cap.isOpened():
            raise IOError("Cannot open webcam")

        # set camera resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        trajectory_points = []
        clear_list = False
        prev_message = None
        count = 0

    while True:
        try:
            if simulation_mode_enabled:
                corners, ids = aruco_detection.getArucoInfo()
            if not simulation_mode_enabled:
                success, img = cap.read()
                # cv2.imshow('image', img)
                corners, ids = findArucoMarkers(img, arucoDict, arucoParam)

            # handle pygame events and keyboard inputs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            topic, message = getKeyboardInput()

            # wait for esc key to be pressed to exit video
            if cv2.waitKey(1) & 0xFF == 27:
                break

            # check if the message is different from the previous one
            if message != prev_message:
                prev_message = message
                zmq_socket.send_string("%d %s" % (topic, message))

            # Get the leader's position and orientation
            leader_center, leader_orientation = get_marker_info(leader_id, ids, corners)
            # print('leader: ', leader_center)

            # Get the follower's position and orientation
            follower_center, follower_orientation = get_marker_info(follower_id, ids, corners)
            # print('follower: ', follower_center)

            # distance between leader and follower
            if not simulation_mode_enabled:
                if follower_center and leader_center:
                    start_point = [float(coord) for coord in follower_center.split()]
                    end_point = [float(coord) for coord in leader_center.split()]
                    leader_orientation_draw = [float(coord) for coord in leader_orientation.split()]
                    follower_orientation_draw = [float(coord) for coord in follower_orientation.split()]
                    trajectory_start_point = np.array(
                        [start_point[0] + 10 * follower_orientation_draw[0], start_point[1] + 10 * follower_orientation_draw[1]])
                    trajectory_end_point = np.array(
                        [end_point[0] - 30 * leader_orientation_draw[0], end_point[1] - 30 * leader_orientation_draw[1]])

                    dx = end_point[0] - start_point[0]
                    dy = end_point[1] - start_point[1]
                    distance = np.sqrt(dx ** 2 + dy ** 2)

                # init flags
                line_calculated = False

            # send leader's position and orientation and the followers position and orientation
            # to the follower only when all information is available and a key is pressed
            if all([leader_center, leader_orientation, follower_center, follower_orientation]) and not message == 'stop' and not message == 'on' and count > 5:
                zmq_socket.send_string("%d %s %s %s %s" % (
                    2, leader_center, leader_orientation, follower_center, follower_orientation))
                count = 0

            if illustration_mode_enabled:
                if distance < 250 and len(trajectory_points) == 0:
                    # Calculate and mark the trajectory points
                    trajectory_points = calculate_trajectory_points(trajectory_start_point, trajectory_end_point)
                elif distance > 250 and len(trajectory_points) == 0:
                    trajectory_points = calculate_points(trajectory_start_point, trajectory_end_point, 4)
                    line_calculated = True

                # clear list when last point in the list is reached
                last_entry = trajectory_points[-1]
                last_entry = np.array(last_entry)
                dx1 = last_entry[0] - start_point[0]
                dy1 = last_entry[1] - start_point[1]
                distance1 = np.sqrt(dx1 ** 2 + dy1 ** 2)

                clear_list = False  # Flag to track whether the list has already been cleared

                # Check if distance is below 50 or distance is below 200 for the first time
                if distance1 < 30 or (distance < 250 and line_calculated):
                    clear_list = True
                    line_calculated = False

                if clear_list:
                    trajectory_points = []  # Clear the list
                    clear_list = False

                mark_points_on_image(img, trajectory_points)

            pygame.display.update()
            if not simulation_mode_enabled:
                cv2.imshow('image', img)

            count += 1

        except IndexError:
            # Ignore IndexError exceptions and continue the loop
            pass

        except TypeError:
            pass

        except Exception as e:
            # Log other types of exceptions and continue the loop
            print("Error:", e)
            pass


if __name__ == "__main__":
    main()
