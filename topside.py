import zmq
import pygame
import sys

def init():
    pygame.init()
    win = pygame.display.set_mode((100,100))


# set up zmq publisher
def setUpPub():
    global socket
    port = "5556"
    if len(sys.argv) > 1:
        port =  sys.argv[1]
        int(port)

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:%s" % port)

def main():
    setUpPub()
    while True:
        for eve in pygame.event.get():
            if eve.type == pygame.KEYDOWN:
                key=pygame.key.name(eve.key)
                if key == "left":
                    topic = 0
                    message = "left"
                    socket.send_string("%d %s" % (topic, message))
                elif key == "right":
                    topic = 0
                    message = "right"
                    socket.send_string("%d %s" % (topic, message))
                elif key == "up":
                    topic = 0
                    message = "straight"
                    socket.send_string("%d %s" % (topic, message))
                elif key == "down":
                    topic = 0
                    message = "back"
                    socket.send_string("%d %s" % (topic, message))
                elif key == "s":
                    topic = 42
                    message = "on"
                    socket.send_string("%d %s" % (topic, message))
                elif key == "q":
                    topic = 42
                    message = "off"
                    socket.send_string("%d %s" % (topic, message))
                    break
            if eve.type == pygame.KEYUP:
                key=pygame.key.name(eve.key)
                topic = 0
                message = "stop"
                socket.send_string("%d %s" % (topic, message))
    

# driver function
if __name__ == "__main__":
    init()
    main()
    