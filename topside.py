import zmq
import pygame
import sys
import aruco_detection


class PygameController:
    def __init__(self, width=100, height=100, port=5556):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:%s" % port)

    def run(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            keys = pygame.key.get_pressed()
            topic = 0
            if keys[pygame.K_s]:
                topic = 42
                message = "on"
            # when 'q' or esc is hit, quit the program
            elif keys[pygame.K_q] or keys[pygame.K_ESCAPE]:
                topic = 42
                message = "off"
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

            self.socket.send_string("%d %s" % (topic, message))

            # Get the leader's position and orientation
            leader_info = aruco_detection.getArucoInfo(0)
            leader_center = leader_info[0]["center"]
            leader_orientation = leader_info[0]["orientation"]

            # Get the follower's position and orientation
            follower_info = aruco_detection.getArucoInfo(1)
            follower_center = follower_info[0]["center"]
            follower_orientation = follower_info[0]["orientation"]

            # send leader's position and orientation and the followers position and orientation
            # to the follower
            self.socket.send_string("%d %s %s %s %s" % (1, leader_center, leader_orientation, follower_center, follower_orientation))

            pygame.display.update()


if __name__ == "__main__":
    controller = PygameController()
    controller.run()
