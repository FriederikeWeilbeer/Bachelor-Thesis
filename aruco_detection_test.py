import cv2
import numpy as np
import mss
import mss.tools
import cv2.aruco as aruco

# set up aruco dictionary
key = getattr(aruco, f'DICT_{6}X{6}_{250}')
arucoDict = aruco.Dictionary_get(key)
arucoParam = aruco.DetectorParameters_create()


def findArucoMarkers(img):
    # convert the video image to gray image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    aruco.drawDetectedMarkers(img, corners)
    return corners, ids


def getArucoInfo(id):
    while True:
        cap = cv2.VideoCapture(1)
        success, img = cap.read()
        corners, ids = findArucoMarkers(img)

        markers = []
        for i in range(len(ids)):
            if ids[i][0] == id:
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

        return markers
