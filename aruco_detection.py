import cv2 as cv
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
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    return corners, ids


def getArucoInfo(id):
    with mss.mss() as sct:
        monitor_number = 2
        mon = sct.monitors[monitor_number]
        # The screen part to capture
        monitor = {
            "top": mon["top"],
            "left": mon["left"],
            "width": mon["width"],
            "height": mon["height"],
            "mon": monitor_number,
        }
        while True:
            img = np.array(sct.grab(monitor))
            corners, ids = findArucoMarkers(img)

            markers = []
            for i in range(len(ids)):
                if ids[i][0] == id:
                    corner_1 = {"x": corners[i][0][0][0], "y": corners[i][0][0][1]}
                    corner_2 = {"x": corners[i][0][1][0], "y": corners[i][0][1][1]}
                    corner_3 = {"x": corners[i][0][2][0], "y": corners[i][0][2][1]}
                    corner_4 = {"x": corners[i][0][3][0], "y": corners[i][0][3][1]}
                    markers.append({"id": id, "corners": [corner_1, corner_2, corner_3, corner_4], "orientation": None})

            return markers
