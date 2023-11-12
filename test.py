from queue import PriorityQueue
from collections import deque

def bfs(graph, start, goal):
    visited = set()
    queue = deque(start)

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node == goal:
            print(f"found path {path}")
            break

        visited.add(node)

        for neighbor in graph[node]:
            if neighbor not in visited:
                neighbor_path = path + neighbor
                queue.append(neighbor_path)

def dfs(graph, start, goal):
    visited= set()
    stack = [start]

    while stack:
        # pop() == pop(-1)
        path = stack.pop()
        node = path[-1]

        if node == goal:
            print(f"found path {path}")
            break

        visited.add(start)

        for neighbor in reversed(graph[node]):
            if neighbor not in visited:
                neighbor_path = path + neighbor
                stack.append(neighbor_path)

def uniform_cost(graph, start, goal):
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set()

    while not queue.empty():
        cost, path = queue.get()
        node = path[-1]

        if node == goal:
            print(f"found path {path} with cost: {cost}")
            break

        visited.add(node)

        for neighbor, neighbor_cost in graph[node].items():
            neighbor_path = path + neighbor
            queue.put((cost + neighbor_cost, neighbor_path))

def greedy(graph, start, goal, h_cost):
    priority_queue = PriorityQueue()
    #put the h_costs in the first pos in the tuple in the pq to make it be sorted based on it
    # h_costs can be put anywhere in the tuple but it will result of an ugly syntax in put()/get()
    priority_queue.put((h_cost[start], start))

    visited = set()

    while priority_queue:
        # node_h_cost is not needed, you can use
        # path = priority_queue.get()[1]
        # or
        # _, path = priority_queue.get()
        # since h_cost is only in the queue for sorting
        node_h_cost, path = priority_queue.get()

        node = path[-1]

        if node == goal:
            print(f"found path {path}")
            break

        visited.add(node)

        for neighbor in graph[node]:
            if neighbor not in visited:
                neighbor_path = path + neighbor
                priority_queue.put((h_cost[neighbor], neighbor_path))

def a_star(graph, start, goal, h_cost):
    priority_queue = PriorityQueue()
    total_cost = 0 + h_cost[start]
    priority_queue.put((total_cost, start, 0))

    visited = set()

    while priority_queue:
        total_cost, path, g_cost = priority_queue.get()

        node = path[-1]

        if node == goal:
            print(f"found path {path}, with total cost {total_cost}")
            break

        visited.add(node)

        for neighbor, neighbor_cost in graph[node].items():
            if neighbor not in visited:
                neighbor_path = path + neighbor
                priority_queue.put((neighbor_cost + g_cost + h_cost[neighbor], neighbor_path, neighbor_cost + g_cost))

# other not important funcions

def bfs_list(graph, start, goal):
    queue = [start]
    visited = set()

    while queue:
        # pop() == pop(-1)
        path = queue.pop(0)
        node = path[-1]

        if node == goal:
            print(f"found {path}")
            break

        visited.add(node)

        for neighbor in graph[node]:
            if neighbor not in visited:
                neighbor_path = path + neighbor
                queue.append(neighbor_path)

def dfs_recursive(graph, start, goal):
    visited = set()

    def _dfs_recursive(path):
        node = path[-1]

        if node == goal:
            print(f"found path {path}")
            return

        visited.add(node)

        for neighbor in graph[node]:
            if neighbor not in visited:
                neighbor_path = path + neighbor
                _dfs_recursive(neighbor_path)

    _dfs_recursive(start)

def uniform_cost_list(graph, start, goal):
    priority_queue = []
    priority_queue.append((0, start))

    visited = set()

    while priority_queue:
        cost, path = priority_queue.pop(0)
        node = path[-1]

        if node == goal :
            print(f"Path found: {path}, Cost: {cost}")
            break

        visited.add(node)

        # here we have to use .items() to get the neighbor and the cost
        for neighbor, neighbor_cost in graph[node].items():
            if neighbor not in visited:
                neighbor_path = path + neighbor
                priority_queue.append((cost + neighbor_cost, neighbor_path))
                priority_queue.sort()



def greedy_list(graph, start, goal, h_cost):
    priority_queue = []
    priority_queue.append((h_cost[start], start))
    visited = set()

    while priority_queue:
        cur_h_cost, path = priority_queue.pop(0)
        node = path[-1]

        if node == goal:
            print(f"found path {path}")
            break

        visited.add(node)

        for neighbor in graph[node]:
            if neighbor not in visited:
                neighbor_path = path + neighbor
                priority_queue.append((h_cost[neighbor], neighbor_path))
                priority_queue.sort()

graph = {
    'A': {'B': 3, 'C': 2},
    'B': {'D': 4, 'E': 1},
    'C': {'B': 1, 'F': 5},
    'D': {'E': 2, 'G': 7},
    'E': {'G': 1},
    'F': {'G': 2},
    'G': {}
}

h_costs = {
    'A': 8,
    'B': 6,
    'C': 4,
    'D': 2,
    'E': 2,
    'F': 1,
    'G': 0
}

bfs(graph, 'A', 'G')
dfs(graph, 'A', 'G')
dfs_recursive(graph, 'A', 'G')
uniform_cost(graph, 'A', 'G')
greedy(graph, 'A', 'G', h_costs)
greedy_list(graph, 'A', 'G', h_costs)
a_star(graph, 'A', 'G', h_costs)















def bfs_commented(graph, start, goal):
    # if start not in graph or goal not in graph:
    #     print("Start or end node not in the graph.")
    #     return
    visited = set()
    queue = deque(start)

    while queue:
        path = queue.popleft()
        #last node in the path to check if it is the goal and to reach it's neighbors
        node = path[-1]

        if node == goal:
            print(f"found {path}")
            break

        visited.add(node)

        #if we did graph[cur].items()
        #we will have to destruct the tuple returned
        # (neighbor, cost) and we don't need the cost
        for neighbor in graph[node]:
            if neighbor not in visited:
                # this snippet/code doesn't work for a graph of numbers, only works for a graph of string/chars
                neighbor_path = path + neighbor
                # to make it a list or to make it work on numbers
                # neighbor_path = list(path)
                # neighbor_path.append(neighbor)
                # we did 'list(path)' and not '[path]' to have a copy not a reference 
                queue.append(neighbor_path)
                # u can use the visited set as the path to every visited node
    print("path not found")

# general template to use
# def search_algorithm(graph, start, goal):
#     initialize data structures (e.g., queue, stack, priority queue)
#     add start node to data structure
#     initialize visited set to track visited nodes
#
#     while data structure is not empty:
#         node = remove node from data structure
#
#         if node == goal:
#             found path to goal
#             break
#
#         visited.add(node)
#
#         for neighbor, neighbor_cost in graph[node].items():
#             if neighbor not in visited:
#                 visited.add(neighbor)
#                 neighbor_path = path + neighbor
#                 add neighbor_path and (accumlated cost or just heuristic cost or both) to data structure

#                     or

#         for neighbor in graph[node]:
#             if neighbor not in visited:
#                 visited.add(neighbor)
#                 neighbor_path = path + neighbor
#                 add neighbor_path to data structure
#
#     path to goal not found

