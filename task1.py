import numpy as np
import cv2
import cv2.aruco as aruco
import socket
import heapq
import time

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
indication = [234]
path_233 = (18, 2)
path_234 = (2, 18)


class Agent():
    def __init__(self, id, maze, order, state=0, test=False) -> None:
        self.id = id
        self.state = state
        self.order = order
        self.position = np.inf
        self.orientation = np.inf
        self.tick = 0
        self.maze = maze.graph
        self.junction_list = maze.get_junction_list()
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

    def stop(self):
        msg = str.encode('t')
        conn_pool[self.order].send(msg)
        if self.id in indication:
            print('Agent {}: stopping..., target:{}'.format(self.id, self.target))
        pass
    
    def forward_offset(self):
        msg = str.encode('f')
        conn_pool[self.order].send(msg)
        print('Agent {}: avoiding..., target:{}'.format(self.id, self.target))
        pass

    def back_offset(self):
        msg = str.encode('b')
        conn_pool[self.order].send(msg)
        print('Agent {}: backing..., target:{}'.format(self.id, self.target))
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

    def is_occupied(self, path):
        self.other_agent = [agent for agent in self.agent_list if self.id != agent.id]
        for i in range(len(path) - 1):
            pair = (path[i], path[i + 1])
            for agent in self.other_agent:
                if agent.target == pair[0] and agent.come_from == pair[1]:
                    self.coming_agent = agent
                    return True
        return False

    def is_junction(self, vertex):
        if vertex in self.junction_list:
            return True
        else:
            return False

    def get_junction_path(self):
        self.junction_path = [self.come_from]
        temp_path = [self.target] + self.path
        for vertex in temp_path:
            self.junction_path.append(vertex)
            if self.is_junction(vertex):
                break

    def set_state(self, new_state):
        self.state = new_state

    def state_control(self):
        '''
        state = 0, initialization
        state = 11, calculate the rotational angle and send the commend
        stage = 12, wait for the movement
        state = 21, calculate the forward distance and send the commend
        stage = 22, wait for the movement
        '''
        if self.state == 0:
            # initialization
            time.sleep(3)
            self.set_state(3)

        if self.state == 11:
            self.forward()
            self.set_state(12)

        if self.state == 12:
            if self.reach(self.target):
                self.come_from = self.target
                if self.path:
                    self.target = self.path.pop(0)
                else:
                    print('else')
                    self.set_state(-1)
                self.get_junction_path()
                if self.is_occupied(self.junction_path):
                    if self.is_junction(self.come_from):
                        self.set_state(41)
                    pass
                else:
                    if self.state != -1:
                        self.set_state(3)
                
            else:
                # self.forward()
                if self.tick % 30 == 0:
                    if self.head_to(self.target):
                        self.set_state(12)
                    else:
                        self.set_state(21)
                else:
                    self.set_state(12)

        if self.state == 21:
            self.turn_to(self.target)
            self.set_state(22)

        if self.state == 22:
            if self.head_to(self.target):
                self.set_state(3)
            else:
                # self.turn_right()
                self.set_state(22)

        if self.state == 3:
            self.stop()
            if self.head_to(self.target):
                self.set_state(11)
            else:
                self.set_state(21)

        if self.state == 41:
            print('{} in State 4！！！！！！！！！！！！！！！'.format(self.id))
            # print('come from:{}, c_path:{}'.format(self.come_from, self.coming_agent.path))
            temp_path = [self.coming_agent.target] + self.coming_agent.path
            for index, vertex in enumerate(temp_path):
                if vertex == self.come_from:
                    coming_agent_target = temp_path[index + 1]
                    break
            forbid_target = (self.target, coming_agent_target)

            for index, val in enumerate(self.maze[self.come_from].ravel()):
                if val != 0 and index not in forbid_target:
                    self.temp_target = index

            print('Agent {} temp_target: {}'.format(self.id, self.temp_target))
            self.turn_to(self.temp_target)
            self.set_state(42)

        if self.state == 42:
            if self.head_to(self.temp_target):
                self.set_state(431)
                self.stop()
                pass
            else:
                self.set_state(42)

        if self.state == 431:
            self.forward_offset()
            self.set_state(432)

        if self.state == 432:
            if self.is_occupied(self.junction_path):
                self.set_state(432)
            else:
                self.target, self.come_from = self.come_from, self.temp_target
                self.set_state(3)

        if self.state == -1:
            self.stop()
            # print('The agent arrives the target')

        self.tick += 1


class Graph():
    def __init__(self, nodes) -> None:
        self.graph = np.zeros((nodes, nodes))
        pass

    def add_edge(self, i, j, w):
        self.graph[i][j] = w
        self.graph[j][i] = w
        pass

    def find_path(self, s, e):
        pqueue = []
        heapq.heappush(pqueue, (0, s))
        seen = set()
        parent = {s: None}
        distance = [np.inf] * self.graph.shape[0]
        distance[s] = 0

        while len(pqueue) > 0:
            pair = heapq.heappop(pqueue)
            dist = pair[0]
            vertex = pair[1]
            seen.add(vertex)

            for i in range(self.graph.shape[0]):
                if self.graph[vertex][i] != 0:
                    if i not in seen:
                        if dist + self.graph[vertex][i] < distance[i]:
                            parent[i] = vertex
                            distance[i] = dist + self.graph[vertex][i]
                            heapq.heappush(pqueue, (distance[i], i))

        come_from = e
        self.path = []
        while come_from is not None:
            self.path.append(come_from)
            come_from = parent[come_from]
        return self.path[::-1]

    def get_junction_list(self):
        self.junction_list = []
        for i in range(self.graph.shape[0]):
            count = 0
            for j in range(self.graph.shape[1]):
                if self.graph[i][j] > 0:
                    count += 1
            if count > 2:
                self.junction_list.append(i)
        
        return self.junction_list


def open_camera():
    cap = cv2.VideoCapture(0)
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


def set_as_maze(graph):
    verticies_1 = (0, 0, 1, 1, 2, 2, 3, 3, 4, 5,
                6, 7, 7, 10, 10, 11, 12, 13, 14, 17, 18, 18, 19)
    verticies_2 = (20, 21, 4, 20, 6, 11, 8, 16, 5,
                    10, 9, 8, 13, 11, 17, 14, 13, 15, 15, 18, 19, 21, 22)

    for vertex_1, vertex_2 in zip(verticies_1, verticies_2):
        graph.add_edge(vertex_1, vertex_2, cal_distance(vertex_1, vertex_2, position))
    return graph

def task_2(agent_list):
    pass


def main():
    mtx, dist = init_parameters()
    cap = open_camera()
    initialization = True

    while True:
        if len(conn_pool) < 2:
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
                        if ids.shape[0] >= 23:
                            initialization = False
                            maze = set_as_maze(Graph(23))
                            path_1 = maze.find_path(path_233[0], path_233[1])
                            path_2 = maze.find_path(path_234[0], path_234[1])
                            print('path_1:', path_1)
                            print('path_2:', path_2)

                            agent_1 = Agent(233, maze, order=0, test=False)
                            agent_2 = Agent(234, maze, order=1, test=False)
                            agent_list = [agent_1, agent_2]

                            for agent, path in zip(agent_list, (path_1, path_2)):
                                agent.set_path(path)
                                agent.set_agent_list(agent_list)
                        else:
                            print('initializing maze...')
            
                    if not initialization:
                        if agent_1.id in position and agent_2.id in position:
                            for agent in agent_list:
                                agent.set_location()
                                agent.set_orientation()
                                agent.state_control()
                        # else:
                        #     print('Distance between node 10 and 17:', cal_distance(10, 17, position))
                            # print('missing agent')

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


if __name__ == "__main__":
    main()