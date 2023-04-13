import zmq
import pygame
import sys
import cv2.aruco as aruco
import cv2
import numpy as np


leader_id = 1
follower_id = 2


def init():
    pygame.init()
    screen = pygame.display.set_mode((100, 100))


def setUpZMQ():
    global socket
    port = "5556"
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:%s" % port)


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


def findArucoMarkers(img, arucoDict, arucoParam):
    # convert the video image to gray image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    aruco.drawDetectedMarkers(img, corners)
    return corners, ids


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


def main():
    global socket
    setUpZMQ()
    # setup aruco Dictionary
    key = getattr(aruco, f'DICT_{6}X{6}_{250}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    # setup video capture
    cap = cv2.VideoCapture(0)

    while True:
        try:
            # get video image
            success, img = cap.read()
            # reading the image
            corners, ids = findArucoMarkers(img, arucoDict, arucoParam)

            # displaying the image
            cv2.imshow('image', img)

            # handle pygame events and keyboard inputs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # get keyboard input
            topic, message = getKeyboardInput()

            # wait for esc key to be pressed to exit video
            if cv2.waitKey(1) & 0xFF == 27:
                break

            # send message to Thymio
            socket.send_string("%d %s" % (topic, message))

            leader_center, leader_orientation = get_marker_info(11, ids, corners)
            print(leader_center, leader_orientation)
            follower_center, follower_orientation = get_marker_info(12, ids, corners)

            if all([leader_center, leader_orientation, follower_center, follower_orientation]):
                socket.send_string("%d %s %s %s %s" % (follower_id, leader_center, leader_orientation, follower_center, follower_orientation))

        except IndexError:
            # Ignore IndexError exceptions and continue the loop
            pass

        except Exception as e:
            # Log other types of exceptions and continue the loop
            print("Error:", e)
            pass

    # close the window
    cv2.destroyAllWindows()


if __name__ == "__main__":
    init()
    main()
