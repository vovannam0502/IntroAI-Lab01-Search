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
    return visited, path

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
    return visited, path

