import zmq
import pygame
import time
import sys
import aruco_detection
import cv2
import cv2.aruco as aruco
import numpy as np


leader_id = 1
follower_id = 2
simulation_mode_enabled = False


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
        message = "off"
    elif keys[pygame.K_s]:
        topic = 42
        message = "on"
    elif keys[pygame.K_UP] and keys[pygame.K_LEFT]:
        message = "left"
    elif keys[pygame.K_UP] and keys[pygame.K_RIGHT]:
        message = "right"
    elif keys[pygame.K_UP]:
        message = "straight"
    elif keys[pygame.K_LEFT]:
        message = "spotleft"
    elif keys[pygame.K_RIGHT]:
        message = "spotright"
    elif keys[pygame.K_DOWN]:
        message = "back"
    else:
        message = "stop"
    return topic, message


def get_marker_info(marker_id, arucoDict, arucoParam, cap):
    if simulation_mode_enabled:
        marker_info = aruco_detection.getArucoInfo(marker_id)
        if marker_info:
            center = marker_info[0]["center"]
            orientation = marker_info[0]["orientation"]
            return center, orientation
    elif not simulation_mode_enabled:

        success, img = cap.read()
        corners, ids = findArucoMarkers(img, arucoDict, arucoParam)
        cv2.imshow('image', img)

        markers = []
        for i, id in enumerate(ids):
            if id == marker_id:
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


def main(screen_size=(100, 100), zmq_port=5556):
    pygame.init()
    screen = pygame.display.set_mode(screen_size)
    setUpZMQ(zmq_port)

    # setup aruco Dictionary
    key = getattr(aruco, f'DICT_{6}X{6}_{250}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    # setup video capture
    cap = cv2.VideoCapture(0)

    while True:
        try:
            # handle pygame events and keyboard inputs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            topic, message = getKeyboardInput()

            # wait for esc key to be pressed to exit video
            if cv2.waitKey(1) & 0xFF == 27:
                break

            zmq_socket.send_string("%d %s" % (topic, message))

            # Get the leader's position and orientation
            leader_center, leader_orientation = get_marker_info(1, arucoDict, arucoParam, cap)

            # Get the follower's position and orientation
            follower_center, follower_orientation = get_marker_info(2, arucoDict, arucoParam, cap)

            # send leader's position and orientation and the followers position and orientation
            # to the follower only when all information is available
            if all([leader_center, leader_orientation, follower_center, follower_orientation]):
                zmq_socket.send_string("%d %s %s %s %s" % (
                    2, leader_center, leader_orientation, follower_center, follower_orientation))

            pygame.display.update()

        except IndexError:
            # Ignore IndexError exceptions and continue the loop
            pass

        except Exception as e:
            # Log other types of exceptions and continue the loop
            print("Error:", e)
            pass


if __name__ == "__main__":
    main()
