# import required packages
import time
import random
import zmq 
import multiprocessing
#import pybonjour
import select
from thymiodirect import Connection 
from thymiodirect import Thymio


# Zeroconf setup
port = 39073
resolved = []
'''
def resolve_callback(sdRef, flags, interfaceIndex, errorCode, fullname, hosttarget, port, txtRecord):
    if errorCode != pybonjour.kDNSServiceErr_NoError:
        return

    ports.append(port)
    resolved.append(True)

def browse_callback(sdRef, flags, interfaceIndex, errorCode, serviceName, regtype, replyDomain):
    if errorCode != pybonjour.kDNSServiceErr_NoError:
        return

    if not (flags & pybonjour.kDNSServiceFlagsAdd):
        print('Service removed')
        return

    resolve_sdRef = pybonjour.DNSServiceResolve(0,
                                                interfaceIndex,
                                                serviceName,
                                                regtype,
                                                replyDomain,
                                                resolve_callback)

    try:
        while not resolved:
            ready = select.select([resolve_sdRef], [], [], 5)
            if resolve_sdRef not in ready[0]:
                print('Resolve timed out')
                break
            pybonjour.DNSServiceProcessResult(resolve_sdRef)
        else:
            resolved.pop()
    finally:
        resolve_sdRef.close()'''


# Robot controller
def stop_robot(robot):
    """Set both wheel robot_speeds to 0 to stop the robot"""
    robot['motor.left.target'] = 0
    robot['motor.right.target'] = 0 

def set_robot_robot_speed(robot, left_robot_speed, right_robot_speed):
    """Set both wheel robot_speeds to the given values"""
    robot['motor.left.target'] = left_robot_speed
    robot['motor.right.target'] = right_robot_speed

def main(use_sim=False, ip='localhost', port=0):
    """main loop of the program"""
    try:
        # Configure Interface to Thymio robot
        if use_sim:                         
            th = Thymio(use_tcp=True, host=ip, tcp_port=port, 
                        on_connect=lambda node_id: print(f' Thymio {node_id} is connected'))
        else:                       
            port = Connection.serial_default_port()
            th = Thymio(serial_port=port, 
                        on_connect=lambda node_id: print(f'Thymio {node_id} is connected'))

        # Robot Connection setup
        th.connect()                        # Connect to the robot
        robot = th[th.first_node()]         # Create am object to control the robot
        print("%s connected" % robot)       # Print the robot name
        time.sleep(5)                       # Delay to allow robot initialization of all variables

        # ZMQ setup
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect('tcp://141.83.194.33:5556')
        #socket.connect('tcp://192.168.8.10:5556')
        socket.setsockopt_string(zmq.SUBSCRIBE, '42')                   # Topic for all robots
        socket.setsockopt_string(zmq.SUBSCRIBE, str(th.first_node()))   # Topic for this robot

        # Initialize variables
        robot_state = 'off'             # State of the robot (on/off)
        robot_action = 'stop'           # Action of the robot
        robot_action_cur = ''           # Current action of the robot
        robot_speed = 500               # Max speed of the robot

        t_action_start = 0              # Time when the current action started
        t_action_duration = 0           # Duration of the current action
        t_pause_duration = 5            # Duration of the pause action
        t_back_duration = 1             # Duration of the back action
        rotation_dir = 0                # Direction of rotation (1/-1)


        print('ready')
        # Main loop
        while True:
            # Receive and handle the message from the ZMQ server
            try:
                topic, data = socket.recv(flags=zmq.NOBLOCK).decode('utf-8').split()
                print(data)

                if topic == '42':                       # Handle the message for all robots
                    robot_state = data
                elif topic == str(th.first_node()):     # Handle the message for this robot
                    robot_action = data
                        
            except zmq.Again as error:
                pass

            
            # Handle the robot state ans set the action
            if robot_state == 'off':    
                robot_action = 'stop'
            elif robot_state == 'on':
                if robot_action == 'stop':
                    robot_action = 'go'
            
                if robot_action == 'pause' and robot_action_cur != 'pause':
                    t_action_duration = t_pause_duration
                    t_action_start = time.time()
                elif robot_action == 'go':
                    for i in range(0, 5):
                        if robot['prox.horizontal'][i] > 3000:
                            robot_action = 'back'
                            t_action_duration = t_back_duration
                            t_action_start = time.time()
                            break
                elif robot_action == 'back':
                    for i in range(5, 7):
                        if robot['prox.horizontal'][i] > 3000:
                            robot_action = 'avoid'
                            t_action_duration = random.uniform(0.5, 1.5)
                            rotation_dir = random.choice([-1, 1])
                            t_action_start = time.time()
                            break


                if t_action_duration > 0:
                    # Check if the current time limited action is finished
                    if time.time() - t_action_start > t_action_duration: 
                        if robot_action == 'back':  # Back action is finished start the avoid action
                            robot_action = 'avoid'
                            t_action_duration = random.uniform(0.5, 1.5)
                            rotation_dir = random.choice([-1, 1])
                            t_action_start = time.time()
                        else:                       # Avoid or pause action is finished start the go action
                            robot_action = 'go'
                            t_action_duration = 0


            # Handle the robot action
            if robot_action == 'go' and robot_action_cur != 'go': 
                robot_action_cur = 'go'         # go straight 
                set_robot_robot_speed(robot, robot_speed, robot_speed)
            elif robot_action == 'avoid' and robot_action_cur != 'avoid': 
                robot_action_cur = 'avoid'      # rotate
                set_robot_robot_speed(robot, -robot_speed*rotation_dir, robot_speed*rotation_dir)
            elif robot_action == 'back' and robot_action_cur != 'back': 
                robot_action_cur = 'back'       # go back
                set_robot_robot_speed(robot, -robot_speed, -robot_speed)
            elif robot_action == 'stop'and robot_action_cur != 'stop':
                robot_action_cur = 'stop'       # stop
                stop_robot(robot)
            elif robot_action == 'pause' and robot_action_cur != 'pause':
                robot_action_cur = 'pause'      # pause 
                stop_robot(robot)


    except Exception as err:
        # Stop robot
        stop_robot(robot)
        print(err)
    except KeyboardInterrupt:
        # Stop robot
        stop_robot(robot)
        print('Keyboard Interrupt')


if __name__ == '__main__':

    # Find robots using Bonjour on _aseba._tcp. service
    '''browse_sdRef = pybonjour.DNSServiceBrowse(regtype="_aseba._tcp.", callBack=browse_callback)

    try:
        # Wait for robots to be found
        ready = select.select([browse_sdRef], [], [])
        if browse_sdRef in ready[0]:
            pybonjour.DNSServiceProcessResult(browse_sdRef)

        print("Found %s robots, Connecting..." % len(ports))

    finally:
        browse_sdRef.close()'''

    print("Starting processes...")

    # spawn process for each robot
    processes = []
    #for port in ports:
    processes.append(multiprocessing.Process(target=main, args=(True, "localhost", port,)))
    
    # start processes
    for p in processes:
        p.start()

    # wait for processes to finish
    for p in processes:
        p.join()
