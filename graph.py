from heapq import heappop

def build_graph(edges: List[LineString], return_close = True): 
    """
    Build graph from set of vertices
    :param edges, list of edges
    :param return_close, a boolean which determines whether we get close nodes dict
    :return adjacency dictionary representing edges of the graph
    """
    adjancency = {}
    close_nodes = {}
    for line in edges:
        p1, p2 = line.coords
        if p1 in adjancency.keys():
            adjancency[p1].append(p2)
        else:
            adjancency[p1] = [p2]
            close_nodes[p1] = False
    #close_nodes = None if not return_close else close_nodes

    return adjancency, close_nodes



class Node:
    def __init__(self, cost, vertex, parent):
        self.cost = cost
        self.vertex = vertex
        self.parent = parent
    def __lt__(self, other):
        return self.cost < other.cost

def Uniform_cost_search(graph, src, dst):
    open = [Node(0, src, None)]
    close = []
    while len(open) != 0:
        next = heappop(open)
        close.append(next)
        if next.vertex == dst.vertex:
            return make_path(next)
        edges = graph[next.vertex]
        for e in edges:
            if e in close:
                continue

            

#build path from destination to source using parent of node
def make_path(final):
    v = final
    path = []
    while v != None:
        path.append(v)
        v = v.parent
    return path[::-1]


#TODO
def get_shorest_path(edges: List[LineString], source=None, dest=None):
    graph = build_graph(edges)
    openheap = [(0,source, None)]
    close_list = []
    

    prev_list = dijkstra(graph, dest, openheap, close_list)
    #rebuild path from prev_list
    path = []
    v = dest
    while v != source:
        path.append(v)
        v = prev_list[v]
    path.append(source)
    path = path[::-1]
    return path
    return LineString(path)
