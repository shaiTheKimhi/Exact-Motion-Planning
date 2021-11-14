import argparse
import os
from typing import List, Tuple

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString
from heapq import heappush, heappop
import numpy as np

from utils.AVL import TreeNode, AVL_Tree


def is_ccw(points):
    points_local = np.concatenate([points, [points[0], points[1]]])
    signed_area = sum(points_local[i,0]*(points_local[i+1,1]-points_local[i-1,1]) for i in range(1, len(points) + 1))/2.0
    return signed_area >= 0.


def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    # creating numpy arrays of polygons points
    points_poly = np.array(original_shape.exterior.xy).T

    # no double first point (start and end are not the same)
    if np.all(points_poly[0] == points_poly[-1]):
        points_poly = points_poly[:-1]

    # make sure not ccw
    if not is_ccw(points_poly):
        points_poly = points_poly[::-1]

    # first point is with lowest y (if more than one then the one with the lowest x as well)
    min_y_points = points_poly[np.where(points_poly[:,1] == points_poly[np.argmin(points_poly[:,1]),1])]
    first_point = min_y_points[np.argmin(min_y_points[:,0])]
    first_index = np.where(np.all(first_point == points_poly, axis=-1))[0][0]

    # initialize with adding the first two points to the end as werr
    points_poly = np.concatenate((points_poly[first_index:], points_poly[:first_index]), axis=0)
    points_poly = np.concatenate([points_poly, [points_poly[0], points_poly[1]]])

    # Because the robot is symmetric, then -R(0,0) = R(0,0)
    points_robot = np.array([[0., -r],
                             [r, 0.],
                             [0., r],
                             [-r, 0.],
                             [0., -r],
                             [r, 0.]])

    # looping to Minkowski sum
    i_poly = 0
    i_robot = 0
    out_points = []
    while i_poly < len(points_poly) - 2 or i_robot < len(points_robot) - 2:
        # update points
        point_poly_curr = points_poly[i_poly]
        point_poly_next = points_poly[i_poly + 1]
        point_robot_curr = points_robot[i_robot]
        point_robot_next = points_robot[i_robot + 1]

        # add point
        out_points.append(point_poly_curr + point_robot_curr)

        # calc angles (when minimum angle=0 and continues after 2*pi)
        angle_poly = np.arctan2(point_poly_next[1] - point_poly_curr[1], point_poly_next[0] - point_poly_curr[0])
        angle_robot = np.arctan2(point_robot_next[1] - point_robot_curr[1], point_robot_next[0] - point_robot_curr[0])
        if angle_poly < 0 or i_poly >= len(points_poly) - 2:
            angle_poly += 2 * np.pi
        if angle_robot < 0 or i_robot >= len(points_robot) - 2:
            angle_robot += 2 * np.pi

        # continue with relevant polygon ccw
        if angle_poly > angle_robot:
            i_robot += 1
        elif angle_poly < angle_robot:
            i_poly += 1
        else:
            i_poly += 1
            i_robot += 1
    out_points = np.array(out_points)
    return Polygon(out_points)

# Check edge is valid in visibility graph
#TODO: improve this check by checking intersection only with edges of polygons (List of LineStrings) which is a point and not one of the given points
def check_edge_validity(obstacles: List[Polygon], edge: LineString):
    for ob in obstacles:
        x = edge.intersection(ob)
        if x.intersects(ob):
            x = edge.intersection(ob)
            if type(x) is not LineString: #intersection is a point
                continue
            sides = LineString(list(ob.exterior.coords))
            x = edge.intersection(sides)
            if type(x) is LineString and len(x.coords) == 2: #intersection is not an edge of the polygon
                continue
            #print(edge.intersection(ob))
            return False
    return True

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
        if p1[0] == -3 and p1[1] == 0:
            for p2 in points:
                if p2 == p1:
                    continue
                # check edge possiblity
                edge = LineString([p1, p2])
                if check_edge_validity(obstacles, edge):
                    check_edge_validity(obstacles, edge)

        for p2 in points: #for duplications elimination we need to take p2 from points[i:] where i is the index of p1 at points
            if p2 == p1:
                continue
            #check edge possiblity
            edge = LineString([p1,p2])
            if check_edge_validity(obstacles, edge):
                edges.append(edge)
    return edges #Until now, works with O(n^3) complexity, could be reduced to O(n^2logn) if sorting obstacles by bounds and ignoring check with non relevant obstacles


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


#TODO: fix algorithm by defining class 'node' with parent and cost, 
def expand(graph, node, openheap, close_nodes, prev={}): 
    """
    Recieves the graph and a certain vertex and pushes list of close vertices to vertex list 
    (two identical vertices will remain only with smallest cost)
    list will be sorted
    :param graph, adjacency dictionary representing graph
    :param node, the vertex to expand and it's cost from source
    :param openheap, the heap of open vertices with their respective costs (from )
    :close_nodes, mapping every node to a boolean stating whether node is closed or open
    """
    cost = node[0]
    vertex = node[1]

    if close_nodes[vertex]: #this is important for vertices to be expanded only by the smallest cost possible
        return close_nodes, prev
    prev = AVL_Tree().insert(prev, vertex, node[2])
    #prev[vertex] = node[2]

    neighbors = graph[vertex]
    for v2 in neighbors:
        #for uniform_cost_search, if vertex is in close list, we would never have to open it again
        #if close_nodes[v2]:
        #    continue
        x1, y1 = vertex[0], vertex[1]
        x2, y2 = v2[0], v2[1]
        c2 = ((x2-x1)**2 + (y2-y1)**2)**0.5
        n = (c2 + cost, v2, vertex) #cost, vertex, previous vertex
        heappush(openheap, n)
    close_nodes = AVL_Tree().insert(close_nodes, vertex, True)
    return close_nodes, prev


def uniform_cost_search(graph, dest, openheap=[], close_nodes={}, prev={}):
    """
    Returns the shortest path from node to dest with given path
    :param graph, adjacency dictionary representing graph
    :param dest, the destination of the search
    :param path, the path traversed in the recursive search
    :param openheap, the heap of open vertices with their respective costs (from )
    :close_list, a list with all nodes that were closed
    """
    if len(openheap) == 0:
        return None, None
    node = heappop(openheap)
    v = node[1] #node is of type (cost,v)
    #if arrived at destination, return path concatenated with last vertex
    if v == dest:
        prev = AVL_Tree().insert(prev, v, node[2])
        #prev[v] = node[2]
        return prev, node[0]
    
    close_nodes, prev = expand(graph, node, openheap, close_nodes, prev) #expand current vertex neighbors

    return uniform_cost_search(graph, dest, openheap, close_nodes, prev) #run uniform_cost_search further until a path is found
    



#since using AVL tree implementation for the close search and prev_list, the time
# complexity is O(n^2 logn) where O(logn) is the time complexity for each iteration
#TODO: make distance function a parameter
def get_shorest_path(edges: List[LineString], source=None, dest=None):
    """
    :param edges: List of LineString elements representing the edges of the graph to search
    :param source: The start point of the robot
    :param dest: The destination of the robot
    :return: List representing the vertices in the shortest path and cost of the shortest path (in distance, could be calculated with a given function)
    """
    graph = build_graph(edges)
    openheap = [(0,source, None)]
    close_nodes = AVL_Tree().insert(None, (float('inf'), 0))
    prev_list = AVL_Tree().insert(None, (float('inf'), 0))

    prev_list, cost = uniform_cost_search(graph, dest, openheap, close_nodes, prev_list)
    if prev_list is None:
        return [], -1
    #rebuild path from prev_list
    path = []
    v = dest
    while v != source:
        path.append(v)
        v = prev_list[v]
    if v != source:
        return [], -1 #path does not exist
    path.append(source)
    path = path[::-1]
    return path, cost
    #return LineString(path)


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
    shortest_path, cost = get_shorest_path(lines, source, dest)

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path)) #this line works only when a path exists


    plotter3.show_graph()