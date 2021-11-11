import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
from heapq import heappush, heappop

# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    pass

# Check edge is valid in visibility graph
#TODO: improve this check by checking intersection only with edges of polygons (List of LineStrings) which is a point and not one of the given points
def check_edge_validity(obstacles: List[Polygon], edge: LineString):
    for ob in obstacles:
        if edge.intersects(ob):
            x = edge.intersection(ob)
            if type(x) is not LineString: #intersection is a point
                continue
            sides = LineString(list(ob.exterior.coords))
            x = edge.intersection(sides)
            if type(x) is LineString: #intersection is not an edge of the polygon
                continue
            #print(edge.intersection(ob))
            return False
    return True


# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    # Possible Optimizations: remove first list, refrain from checking points in the same polygon,
    # refrain from checking intersection with polygons out of our bounds (requires sorting polygon)
    edges = []
    points = []
    points += [source] if source is not None else []
    points += [dest] if dest is not None else []

    for ob in obstacles:
        points += list(ob.exterior.coords)
    for p1 in points:
        for p2 in points: #for duplications elimination we need to take p2 from points[i:] where i is the index of p1 at points
            if p2 == p1:
                continue
            #check edge possiblity
            edge = LineString([p1,p2])
            if check_edge_validity(obstacles, edge):
                edges.append(edge)
    return edges #Until now, works with O(n^3) complexity, could be reduced to O(n^2logn) if sorting obstacles by bounds and ignoring check with non relevant obstacles

#TODO
def build_graph(edges: List[LineString]):
    """
    Build graph from set of vertices
    :param edges, list of edges
    :return adjacency dictionary representing edges of the graph
    """
    adjancency = {}
    for line in edges:
        p1, p2 = line.coords
        if p1 in adjancency.keys():
            adjancency[p1].append(p2)
        else:
            adjancency[p1] = [p2]

    return adjancency



def expand(graph, node, openheap, close_list, prev={}): 
    """
    Recieves the graph and a certain vertex and pushes list of close vertices to vertex list 
    (two identical vertices will remain only with smallest cost)
    list will be sorted
    :param graph, adjacency dictionary representing graph
    :param node, the vertex to expand and it's cost from source
    :param openheap, the heap of open vertices with their respective costs (from )
    :close_list, a list with all nodes that were closed
    """
    cost = node[0]
    vertex = node[1]
    if vertex in close_list:
        return
    prev[vertex] = node[2]

    neighbors = graph[vertex]
    for v2 in neighbors:
        #for dijkstra, if vertex is in close list, we would never have to open it again
        if v2 in close_list:
            continue
        x1, y1 = vertex[0], vertex[1]
        x2, y2 = v2[0], v2[1]
        c2 = ((x2-x1)**2 + (y2-y1)**2)**0.5
        n = (c2 + cost, v2, vertex) #cost, vertex, previous vertex
        heappush(openheap, n)
    close_list.append(vertex)


def dijkstra(graph, dest, openheap=[], close_list=[], prev={}):
    """
    Returns the shortest path from node to dest with given path
    :param graph, adjacency dictionary representing graph
    :param dest, the destination of the search
    :param path, the path traversed in the recursive search
    :param openheap, the heap of open vertices with their respective costs (from )
    :close_list, a list with all nodes that were closed
    """
    node = heappop(openheap)
    v = node[1] #node is of type (cost,v)
    #if arrived at destination, return path concatenated with last vertex
    if v == dest:
        prev[v] = node[2]
        return prev
    
    ##TODO: we have a problem, node to open must consist of full path as well as cost

    expand(graph, node, openheap, close_list, prev) #expand current vertex neighbors

    return dijkstra(graph, dest, openheap, close_list, prev) #run dijkstra further until a path is found
    



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


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line (graph search for shortest path withing the graph)
    shortest_path, cost = None, None

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))


    plotter3.show_graph()