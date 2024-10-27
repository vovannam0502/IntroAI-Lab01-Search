import numpy as np


def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
   
    path=[]
    visited={}
    node_before = {} # Danh sách các node trước đó của node hiện tại

    queue = [start] # Queue chứa các node cần thăm, push start vào queue
    visited[start] = None # start là node đầu tiên nên node thăm trước đó là None
    node_before[start] = None 

    while queue:
        node = queue.pop(0) # Lấy node đầu tiên ra khỏi queue (thăm đỉnh)
        visited[node] = node_before[node] # Đánh dấu node đã thăm

        # Nếu node vừa thăm là end thì dừng tìm kiếm
        if node == end:
            break

        # Thêm các node kề của node vừa thăm vào queue
        for i in range(len(matrix[node])):
            # Nếu có cạnh nối từ node đang xét đến node i và node i chưa được thăm, đánh dấu node trước đó của node i là node đang xét
            # Push node i vào queue
            if matrix[node][i] and i not in visited and i not in queue:
                node_before[i] = node
                queue.append(i)

    # Kiểm tra xem node end có trong visited không
    if end in visited:
        node = end
        # Truy ngược lại từ end để tìm path
        while node is not None:
            path.insert(0, node)
            node = visited[node]
    else:
        path = []  # Nếu không tìm thấy đường đi thì trả về đường đi rỗng
  
    # In ra visited và path để kiểm tra
    print(visited)
    print(path)

    return visited, path

def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    
    path=[]
    visited={}
    node_before = {} # Danh sách các node trước đó của node hiện tại

    # Sử dụng stack để thực hiện DFS
    stack = [start] # stack chứa các node cần thăm, push start vào stack
    visited[start] = None # start là node đầu tiên nên node thăm trước đó là None
    node_before[start] = None

    while stack:
        node = stack.pop() # Lấy node cuối cùng ra khỏi stack (thăm đỉnh)

        visited[node] = node_before[node] # Đánh dấu node đã thăm

        # Nếu node vừa thăm là end thì dừng tìm kiếm
        if node == end:
            break
        
        # Thêm các node kề của node vừa thăm vào stack
        for i in range(len(matrix[node])):
            # Nếu có cạnh nối từ node đang xét đến node i và node i chưa được thăm, đánh dấu node trước đó của node i là node đang xét
            # Push node i vào stack
            if matrix[node][i] and i not in visited:
                node_before[i] = node
                stack.append(i)

    # Kiểm tra xem node end có trong visited không
    if end in visited:
        node = end
        # Truy ngược lại từ end để tìm path
        while node is not None:
            path.insert(0, node)
            node = visited[node]
    else:
        path = []  # Nếu không tìm thấy đường đi thì trả về đường đi rỗng
  
    # In ra visited và path để kiểm tra
    print(visited)
    print(path)
   
    return visited, path


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}
    dist=[float('inf')] * len(matrix) # dist là khoảng cách từ start đến các đỉnh khác, đặt giá trị ban đầu là vô cực

    queue = [(0, start, None)] # Đỉnh bắt đầu là start. (dist, node, node_before) của start là (0, start, None)
    dist[start] = 0

    while queue:
        queue.sort(key=lambda x: x[0]) # Sắp xếp queue theo dist, dist nhỏ nhất ở đầu queue
        u = queue.pop(0) # Lấy ra đỉnh có dist nhỏ nhất

        dist_node, node, node_before = u[0], u[1], u[2]

        # Nếu node đã được thăm thì bỏ qua
        if node in visited:
            continue

        visited[node] = node_before # Đánh dấu node đã thăm, cập nhật node trước của node đó

        # Nếu node vừa thăm là end thì dừng tìm kiếm
        if node == end:
            break
        
        # Duyệt qua các đỉnh kề của node vừa thăm
        for i in range(len(matrix[node])):
            # Nếu có cạnh nối từ node đang xét đến node i và node i chưa được thăm
            if matrix[node][i] and i not in visited:
                # Cập nhật dist[i] nếu có cách đi tốt hơn
                # Thêm (dist, node, node_before) của i vào queue
                if dist[i] > dist[node] + matrix[node][i]:
                    dist[i] = dist[node] + matrix[node][i]
                    queue.append((dist[i], i, node))
        
    # Kiểm tra xem node end có trong visited không
    if end in visited:
        node = end
        # Truy ngược lại từ end để tìm path
        while node is not None:
            path.insert(0, node)
            node = visited[node]
    else:
        path = []  # Nếu không tìm thấy đường đi thì trả về đường đi rỗng

    # In ra visited và path để kiểm tra
    print(visited)
    print(path)

    return visited, path

# Thuật toán Greedy Best First Search sử dụng hàm heuristic h = edge weight
def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}

    queue = [(0, start, None)] # (h, node, node_before) của start là (0, start, None)

    while queue:
        queue.sort(key=lambda x: x[0]) # Sắp xếp queue theo h, h nhỏ nhất ở đầu queue
        u = queue.pop(0) # Lấy ra đỉnh có h nhỏ nhất

        h, node, node_before = u[0], u[1], u[2]

        # Nếu node đã được thăm thì bỏ qua
        if node in visited:
            continue

        visited[node] = node_before # Đánh dấu node đã thăm, cập nhật node trước của node đó

        # Nếu node vừa thăm là end thì dừng tìm kiếm
        if node == end:
            break

        # Duyệt qua các đỉnh kề của node vừa thăm
        for i in range(len(matrix[node])):
            # Nếu có cạnh nối từ node đang xét đến node i và node i chưa được thăm
            if matrix[node][i] and i not in visited:
                h = matrix[node][i] # h là edge weight từ node đang xét đến node i
                queue.append((h, i, node)) # Thêm (h, node, node_before) của i vào queue
        
    # Kiểm tra xem node end có trong visited không
    if end in visited:
        node = end
        # Truy ngược lại từ end để tìm path
        while node is not None:
            path.insert(0, node)
            node = visited[node]
    else:
        path = []  # Nếu không tìm thấy đường đi thì trả về đường đi rỗng

    # In ra visited và path để kiểm tra
    print(visited)
    print(path)

    return visited, path

# Hàm heuristic cho A* sử dụng Euclidean distance
# h = sqrt((x1 - x2)^2 + (y1 - y2)^2)
def eclidean_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

# Thuật toán A* sử dụng hàm heuristic h = eclidean_distance(pos[current vertex], pos[Goal])
def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 

    path=[]
    visited={}
    dist=[float('inf')] * len(matrix) # dist[i] là khoảng cách từ start đến đỉnh i, đặt giá trị ban đầu là vô cực

    queue = [(0, start, None)] # (f, node, node_before) của start là (0, start, None), f = g + h
    # g là khoảng cách từ start đến đỉnh đang xét, h là khoảng cách từ đỉnh đang xét đến end
    dist[start] = 0

    while queue:
        queue.sort(key=lambda x: x[0]) # Sắp xếp queue theo f, f nhỏ nhất ở đầu queue
        u = queue.pop(0) # Lấy ra đỉnh có f nhỏ nhất

        f, node, node_before = u[0], u[1], u[2]
        
        # Nếu node đã được thăm thì bỏ qua
        if node in visited:
            continue

        visited[node] = node_before # Đánh dấu node đã thăm, cập nhật node trước của node đó

        # Nếu node vừa thăm là end thì dừng tìm kiếm
        if node == end:
            break

        # Duyệt qua các đỉnh kề của node vừa thăm
        for i in range(len(matrix[node])):
            # Nếu có cạnh nối từ node đang xét đến node i và node i chưa được thăm
            if matrix[node][i] and i not in visited:
                if dist[i] > dist[node] + matrix[node][i]: # Cập nhật dist[i] nếu có cách đi tốt hơn
                    dist[i] = dist[node] + matrix[node][i]
                
                g = dist[i] # g là khoảng cách từ start đến đỉnh đang xét, dist[i]
                h = eclidean_distance(pos[i], pos[end]) # h là khoảng cách (heuristic) từ đỉnh đang xét đến end
                f = g + h # f = g + h
                queue.append((f, i, node)) # Thêm (f, node, node_before) của i vào queue 

    # Kiểm tra xem node end có trong visited không
    if end in visited:
        node = end
        # Truy ngược lại từ end để tìm path
        while node is not None:
            path.insert(0, node)
            node = visited[node]
    else:
        path = []  # Nếu không tìm thấy đường đi thì trả về đường đi rỗng

    # In ra visited và path để kiểm tra
    print(visited)
    print(path)

    return visited, path

