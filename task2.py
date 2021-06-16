import json
import numpy as np
import cv2
from numpy.core.records import array
import cv2.aruco as aruco
import socket
from urllib.request import urlopen
from get_img import get_img as gi

ADDRESS = ('', 10000)
central = None
conn_pool = []

central = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
central.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
central.setblocking(False)
central.bind(ADDRESS)
central.listen(5)
print("Waiting...")

position = {}
orientation = {}
temp_threshold = 40
indication = [233]


font = cv2.FONT_HERSHEY_SIMPLEX
green_lower = np.array([35, 110, 106])
green_upper = np.array([77, 255, 255])
red_lower = np.array([156, 43, 46])
red_upper = np.array([180, 255, 255])
yellow_lower = np.array([26, 43, 46])
yellow_upper = np.array([34, 255, 255])

bts = b''
fix_size = (640, 480)
CAMERA_BUFFRER_SIZE = 8192

class Agent():
    def __init__(self, id, order, state=0, test=False) -> None:
        self.id = id
        self.state = state
        self.order = order
        self.position = np.inf
        self.orientation = np.inf
        self.tick = 0
        self.come_from = str(self.id) + 'come_from'
        self.target = str(self.id) + 'target'
        self.flag = True
        self.url = 'http://192.168.1.27:81/stream'
        if test:
            self.path = [15, 16]
        pass

    def set_location(self):
        if self.id in position:
            self.position = position[self.id]

    def set_orientation(self):
        if self.id in orientation:
            self.orientation = orientation[self.id]

    def set_path(self, path):
        self.path = path
        self.come_from = self.path.pop(0)
        self.target = self.path.pop(0)

    def set_agent_list(self, agent_list):
        self.agent_list = agent_list

    def forward(self):
        msg = str.encode('w')
        conn_pool[self.order].send(msg)
        if self.id in indication:
            print('Agent {}: forward..., target:{}'.format(self.id, self.target))
        pass

    def backward(self):
        msg = str.encode('s')
        conn_pool[self.order].send(msg)
        if self.id in indication:
            print('Agent {}: backward..., target:{}'.format(self.id, self.target))
        pass

    def turn_right(self):
        msg = str.encode('d')
        conn_pool[self.order].send(msg)
        if self.id in indication:
            print('Agent {}: right..., target:{}'.format(self.id, self.target))
        pass

    def turn_left(self):
        msg = str.encode('a')
        conn_pool[self.order].send(msg)
        if self.id in indication:
            print('Agent {}: left..., target:{}'.format(self.id, self.target))
        pass

    def turn_to(self, target):
        v1 = position[target] - position[self.id]
        v2 = np.array([1, 0])
        cos_angle = v1.dot(v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(cos_angle) / np.pi * 180
        if v1[1] < 0:
            angle *= -1
        agent_ori = self.orientation
        # print(angle)
        # print(agent_ori)
        if abs(angle - agent_ori) > 180:
            if angle > agent_ori:
                self.turn_left()
            else:
                self.turn_right()
        else:
            if angle < agent_ori:
                self.turn_left()
            else:
                self.turn_right()

    def turn_to_ori(self, angle):
        agent_ori = self.orientation
        # print(angle)
        # print(agent_ori)
        if abs(angle - agent_ori) > 180:
            if angle > agent_ori:
                self.turn_left()
            else:
                self.turn_right()
        else:
            if angle < agent_ori:
                self.turn_left()
            else:
                self.turn_right()

    def stop(self):
        msg = str.encode('t')
        conn_pool[self.order].send(msg)
        if self.id in indication:
            print('Agent {}: stopping..., target:{}'.format(self.id, self.target))
        pass
    
    def look_for_target(self):
        msg = str.encode('o')
        conn_pool[self.order].send(msg)
        pass

    def thermal(self):
        msg = str.encode('l')
        conn_pool[self.order].send(msg)
        pass

    def attack(self):
        msg = str.encode('k')
        conn_pool[self.order].send(msg)
        print('Agent {} is attacking!!'.format(self.id))
        pass

    def get_img(self):
        return gi(self.url)

    def find_edge(self):
        msg = str.encode('g')
        conn_pool[self.order].send(msg)
        pass

    def circle(self):
        msg = str.encode('r')
        conn_pool[self.order].send(msg)
        pass

    def quit(self):
        msg = str.encode('q')
        conn_pool[self.order].send(msg)
        pass

    def reach(self, target):
        if cal_distance(target, self.id, position) < 0.04:
            return True
        else:
            return False

    def head_to(self, id):
        v1 = position[id] - position[self.id]
        v2 = np.array([1, 0])
        cos_angle = v1.dot(v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(cos_angle) / np.pi * 180
        if v1[1] < 0:
            angle *= -1

        if self.orientation - angle < 3 and self.orientation - angle > -3:
            return True
        else:
            return False

    def head_to_ori(self, angle):
        if abs(self.orientation - angle) < 12:
            return True
        else:
            return False

    def set_state(self, new_state):
        self.state = new_state

    def state_control_2(self):
        if self.state == 0:
            if self.id == 233:
                self.set_state(11)
            else:
                self.set_state(0)

        if self.state == 10:
            # initialization

            self.set_state(93)

        if self.state == 911:
            self.forward()
            self.set_state(912)

        if self.state == 912:
            if self.reach(self.target):
                self.set_state(-1)
            else:
                if self.tick % 30 == 0:
                    if self.head_to(self.target):
                        self.set_state(912)
                    else:
                        self.set_state(921)
                else:
                    self.set_state(912)

        if self.state == 921:
            self.turn_to(self.target)
            self.set_state(922)

        if self.state == 922:
            if self.head_to(self.target):
                self.set_state(93)
            else:
                # self.turn_right()
                self.set_state(922)

        if self.state == 93:
            self.stop()
            if self.head_to(self.target):
                self.set_state(911)
            else:
                self.set_state(921)
        
        if self.state == 11:
            self.look_for_target()
            self.set_state(12)
        
        if self.state == 12:
            try:
                data = conn_pool[self.order].recv(1064)
                if len(data) != 0:
                    msg = data.decode('utf-8')
                    print(msg)
                    if msg == 'Reach the object':
                        self.set_state(21)
            except Exception:
                # print('12 except')
                self.set_state(12)
                pass

        if self.state == 21:
            self.thermal()
            self.set_state(22)

        if self.state == 22:
            try:
                data = conn_pool[self.order].recv(1064)
                json_string = json.loads(data)
                self.array = format_thermal(json_string)
                print(self.array)
                self.set_state(23)
            except Exception:
                # print('22 except')
                self.set_state(22)
                pass
        
        if self.state == 23:
            self.max_temp = max(max(self.array))
            if  self.max_temp == 0:
                self.set_state(21)
            else:
                self.set_state(24)

        if self.state == 24:
            if self.max_temp > temp_threshold:
                self.set_state(31)
            else:
                self.set_state(41)

        if self.state == 31:
            self.find_edge()
            self.set_state(32)
        
        if self.state == 32:
            try:
                data = conn_pool[self.order].recv(1064)
                self.edge_len = float(data.decode('utf-8'))
                print('edge length:', self.edge_len)
                position['start'] = position[self.id]
                self.forward()
                self.set_state(33)
            except Exception:
                self.set_state(32)
                pass
        
        if self.state == 33:
            # print('distance: ', cal_distance(self.id, 'start'))
            if cal_distance(self.id, 'start') < 0.5:
                self.set_state(33)
            else:
                position[str(self.id) + 'come_from'] = position[self.id]
                self.set_state(10)

        if self.state == 41:
            color = self.get_img()
            position['obj'] = position[self.id]
            orientation['obj'] = orientation[self.id]
            if color == 'red':
                print('Red!!!!!')
                self.agent_list[2].set_state(61)
                self.set_state(10)
                pass
            elif color == 'yellow':
                print('Yellow!!!!!')
                self.agent_list[1].set_state(61)
                self.set_state(10)
                pass
            elif color == 'green':
                print('Green!!!!!')
                self.set_state(51)
                pass
            else:
                self.set_state(41)
                pass
        
        if self.state == 51:
            self.circle()
            self.set_state(52)

        if self.state == 52:
            try:
                data = conn_pool[self.order].recv(1064)
                msg = data.decode('utf-8')
                if msg == 'Complete':
                    self.set_state(-1)
            except Exception:
                self.set_state(52)
                pass
        
        if self.state == 61:
            position[str(self.id) + 'target'] = position['obj']
            self.set_state(10)

        if self.state == -1:
            if self.id == 233:
                self.stop()
            else:
                self.set_state(-21)
            pass

        if self.state == -21:
            self.turn_to_ori(orientation['obj'])
            self.set_state(-22)
            pass

        if self.state == -22:
            if self.head_to_ori(orientation['obj']):
                self.set_state(-23)
            else:
                self.set_state(-22)

        if self.state == -23:
            self.forward()
            self.set_state(-24)

        if self.state == -24:
            if self.head_to_ori(orientation['obj']):
                if cal_distance('obj', self.id) >= 0.9:
                    self.set_state(-4)
                else:
                    self.set_state(-24)
            else:
                self.set_state(-31)
            # if cal_distance('obj', self.id) >= 1:
            #     self.set_state(-4)
            # else:
            #     self.set_state(-24)
        
        if self.state == -31:
            self.turn_to_ori(orientation['obj'])
            self.set_state(-32)

        if self.state == -32:
            print('Ori: {}, OBJ_ori: {}'.format(self.orientation, orientation['obj']))
            if self.head_to_ori(orientation['obj']):
                self.set_state(-23)
            else:
                self.set_state(-32)

        if self.state == -4:
            self.stop()
            self.attack()

        if self.tick % 50 ==0:
            if self.id in indication:
                print(str(self.id) + ' state: ' + str(self.state))
        self.tick += 1
 

def open_camera():
    cap = cv2.VideoCapture(1)
    cap.set(3, 1920)
    cap.set(4, 1080)
    return cap


def init_parameters():
    mtx = np.array([[1051.1, 0, 695.0741],
                    [0, 1052.2, 297.7604],
                    [0., 0., 1.]])
    dist = np.array([[-0.4223, 0.1412, 0, 0, 0.0921]])
    return mtx, dist


def capture_frame(cap):
    ret, frame = cap.read()
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    return frame


def detect_aruco(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    corners, ids, rIP = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids, rIP

def get_position(ids, tvec, position):
    for i in range(ids.shape[0]):
        position[ids[i][0]] = (tvec[i][0])[:2]


def get_orientation(ids, rvec, orientation):
    for i in range(ids.shape[0]):
        temp = rvec[i][0]
        r, _ = cv2.Rodrigues(temp)
        theta_z = np.arctan2(r[1][0], r[0][0]) / np.pi * 180
        orientation[ids[i][0]] = theta_z


def cal_distance(id1, id2, pos=position):
    if id1 in pos and id2 in pos:
        distance = np.linalg.norm(pos[id1] - pos[id2])
        return distance
    else:
        return np.inf


def cal_angle(agent, vertex_id, next_id, pos):
    try:
        vertex = pos[vertex_id]
        next = pos[next_id]
        v1 = agent.position - vertex
        v2 = next - vertex
        cos_angle = v1.dot(v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angle = np.arccos(cos_angle) / np.pi * 180
        return angle
    except Exception:
        return np.inf


def format_thermal(one_d_array):
    two_d_array = []
    i = 0
    for row in range(8):
        temp = []
        for col in range(8):
            temp.append(one_d_array[i])
            i = i + 1
        two_d_array.append(temp)
    return two_d_array


def main():
    mtx, dist = init_parameters()
    cap = open_camera()
    initialization = True

    while True:
        if len(conn_pool) < 3:
            try:
                client, _ = central.accept()
                # print('address: {}，port: {} is connected'.format(addr[0], addr[1]))
                conn_pool.append(client)
            except BlockingIOError:
                pass
        else:
            try:
                frame = capture_frame(cap)
                corners, ids, _ = detect_aruco(frame)

                if ids is not None:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.158, mtx, dist)
                
                    for i in range(rvec.shape[0]):
                        aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.1)
                    aruco.drawDetectedMarkers(frame, corners, ids, (0, 0, 255))

                    get_position(ids, tvec, position)
                    get_orientation(ids, rvec, orientation)

                    if initialization:
                        if ids.shape[0] >= 4:
                            initialization = False
                            agent_1 = Agent(233, order=0, state=21)
                            agent_2 = Agent(234, order=1)
                            agent_3 = Agent(235, order=2)
                            agent_list = [agent_1, agent_2, agent_3]
                            for agent_id, id in zip((agent_1.id, agent_2.id, agent_3.id), (101, 102, 103)):
                                position[str(agent_id) + 'come_from'] = position[id]
                                position[str(agent_id) + 'target'] = position[104]
                            for agent in agent_list:
                                agent.set_agent_list(agent_list)
                            print('initialization complete...')
                        else:
                            print('initializing...')
            
                    if not initialization:
                        if agent_1.id in position and agent_2.id in position and agent_3.id in position:
                            for agent in agent_list:
                                agent.set_location()
                                agent.set_orientation()
                                agent.state_control_2()
                    

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    for agent in agent_list:
                        agent.stop()
                    agent.quit()
                    break
                cv2.imshow("Capture", frame)

            except(BlockingIOError, ConnectionResetError):
                print("Error 2")
                pass

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()