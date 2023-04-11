import zmq
import pygame
import time
import sys
import aruco_detection


def get_marker_info(marker_id):
    marker_info = aruco_detection.getArucoInfo(marker_id)
    if marker_info:
        center = marker_info[0]["center"]
        orientation = marker_info[0]["orientation"]
        return center, orientation
    else:
        return None, None


class RobotController:
    def __init__(self, screen_size=(100, 100), zmq_port=5556):
        pygame.init()
        self.screen = pygame.display.set_mode(screen_size)
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://*:%s" % zmq_port)

    def run(self):
        while True:
            try:
                # handle pygame events and keyboard inputs
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()

                keys = pygame.key.get_pressed()
                topic = 0
                # when 'q' or esc is hit, quit the program

                if keys[pygame.K_ESCAPE]:
                    # send the message to all robots
                    self.zmq_socket.send_string("%d %s" % (42, "quit"))
                    time.sleep(5)
                    print("Quitting...")
                    sys.exit()
                elif keys[pygame.K_q]:
                    topic = 42
                    message = "off"
                    # send the message to all robots
                    self.zmq_socket.send_string("%d %s" % (topic, message))
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

                self.zmq_socket.send_string("%d %s" % (topic, message))

                # Get the leader's position and orientation
                leader_center, leader_orientation = get_marker_info(0)

                # Get the follower's position and orientation
                follower_center, follower_orientation = get_marker_info(1)

                # send leader's position and orientation and the followers position and orientation
                # to the follower only when all information is available
                if all([leader_center, leader_orientation, follower_center, follower_orientation]):
                    self.zmq_socket.send_string("%d %s %s %s %s" % (1, leader_center, leader_orientation, follower_center, follower_orientation))

                pygame.display.update()

            except IndexError:
                # Ignore IndexError exceptions and continue the loop
                pass

            except Exception as e:
                # Log other types of exceptions and continue the loop
                print("Error:", e)
                pass


if __name__ == "__main__":
    controller = RobotController()
    controller.run()
