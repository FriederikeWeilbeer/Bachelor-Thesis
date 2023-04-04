import cv2 as cv
import numpy as np
import mss
import mss.tools
import cv2.aruco as aruco

DEBUG = False


def findArucoMarkers(img, arucoDict, arucoParam):
    # convert the video image to a gray image
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)
    # draw boxes around detected markers
    # aruco.drawDetectedMarkers(img, corners)
    return corners, ids


def getPosition(corners, ids):
    try:
        for i in range(0, len(ids)):
            return (str(ids[i][0]) +
                    " " + str(corners[i][0][0][0]) + " " + str(corners[i][0][0][1]) +
                    " " + str(corners[i][0][1][0]) + " " + str(corners[i][0][1][1]) +
                    " " + str(corners[i][0][2][0]) + " " + str(corners[i][0][2][1]) +
                    " " + str(corners[i][0][3][0]) + " " + str(corners[i][0][3][1]))
    except Exception as err:
        if DEBUG:
            print("No markers found.")


def main():
    # setup aruco Dictionary
    key = getattr(aruco, f'DICT_{6}X{6}_{250}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()

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
        # monitor = {"top": 40, "left": 0, "width": 800, "height": 640}
        while True:
            # Get raw pixels from the screen, save it to a Numpy array
            img = np.array(sct.grab(monitor))
            corners, ids = findArucoMarkers(img, arucoDict, arucoParam)

            # Display the picture in grayscale
            cv.imshow('Aruco detection', img)
            print(getPosition(corners, ids))

            # Press "q" to quit
            if cv.waitKey(25) & 0xFF == ord("q"):
                cv.destroyAllWindows()
                break


# driver function
if __name__ == "__main__":
    main()
