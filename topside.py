import zmq
import pygame
import sys
import mss


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
            pygame.display.update()



if __name__ == "__main__":
    controller = PygameController()
    controller.run()
