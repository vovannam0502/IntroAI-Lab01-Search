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
    node_before = {}

    queue = [start]
    visited[start] = None
    node_before[start] = None

    while queue:
        node = queue.pop(0)
        visited[node] = node_before[node]

        if node == end:
            break

        for i in range(len(matrix[node])):
            if matrix[node][i] and i not in visited:
                node_before[i] = node
                queue.append(i)

    node = end
    # Truy ngược lại từ end để tìm path
    while node != None:
        path.insert(0, node)
        node = visited[node]
  
    # In ra visited và path
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
    node_before = {}

    stack = [start]
    visited[start] = None
    node_before[start] = None

    while stack:
        node = stack.pop()
        visited[node] = node_before[node]

        if node == end:
            break

        for i in range(len(matrix[node])):
            if matrix[node][i] and i not in visited:
                node_before[i] = node
                stack.append(i)

    node = end
    # Truy ngược lại từ end để tìm path
    while node != None:
        path.insert(0, node)
        node = visited[node]
  
    # In ra visited và path
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
    dist=[float('inf')] * len(matrix)

    queue = [(0, start, None)]
    dist[start] = 0

    while queue:
        queue.sort(key=lambda x: x[0])
        u = queue.pop(0) # u[0], u[1], u[2] lần lượt là dist, node, node_before

        if u[1] in visited:
            continue

        visited[u[1]] = u[2]

        if u[1] == end: # u[1] là đỉnh vừa thăm
            break
        
        for i in range(len(matrix[u[1]])):
            if matrix[u[1]][i] and i not in visited:
                if dist[i] > dist[u[1]] + matrix[u[1]][i]:
                    dist[i] = dist[u[1]] + matrix[u[1]][i]
                    queue.append((dist[i], i, u[1]))
        
    node = end
    # Truy ngược lại từ end để tìm path
    while node != None:
        path.insert(0, node)
        node = visited[node]

    # In ra visited và path
    print(visited)
    print(path)

    return visited, path


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

    queue = [(0, start, None)]

    while queue:
        queue.sort(key=lambda x: x[0])
        u = queue.pop(0) # đỉnh có cạnh nhỏ nhất

        if u[1] in visited:
            continue
        visited[u[1]] = u[2]

        if u[1] == end: # u[1] là đỉnh vừa thăm
            break

        for i in range(len(matrix[u[1]])):
            if matrix[u[1]][i] and i not in visited:
                queue.append((matrix[u[1]][i], i, u[1]))
        
    node = end
    # Truy ngược lại từ end để tìm path
    while node != None:
        path.insert(0, node)
        node = visited[node]

    # In ra visited và path
    print(visited)
    print(path)

    return visited, path

# Hàm heuristic cho A* sử dụng khoảng cách Euclidean giữa 2 điểm
def eclidean_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

# Thuật toán A*
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
    dist=[float('inf')] * len(matrix)

    queue = [(0, start, None)] # (f, node, node_before), f = g + h
    dist[start] = 0

    while queue:
        queue.sort(key=lambda x: x[0])
        u = queue.pop(0) 
        
        if u[1] in visited:
            continue
        visited[u[1]] = u[2]

        if u[1] == end:
            break

        for i in range(len(matrix[u[1]])):
            if matrix[u[1]][i] and i not in visited:
                if dist[i] > dist[u[1]] + matrix[u[1]][i]: # nếu có cách đi tốt hơn
                    dist[i] = dist[u[1]] + matrix[u[1]][i] # cập nhật dist (dist là khoảng cách từ start đến đỉnh đang xét)
                
                g = dist[i] # g là khoảng cách từ start đến đỉnh đang xét
                h = eclidean_distance(pos[i], pos[end]) # h là khoảng cách (heuristic) từ đỉnh đang xét đến end
                f = g + h # f = g + h
                queue.append((f, i, u[1])) # thêm vào queue

    node = end
    # Truy ngược lại từ end để tìm path
    while node != None:
        path.insert(0, node)
        node = visited[node]

    # In ra visited và path
    print(visited)
    print(path)

    return visited, path

