#This test is my test, don't need to keep it
from HW1 import get_visibility_graph, get_shorest_path, build_graph
from json import loads, dumps
from shapely.geometry.polygon import Polygon, LineString


p1 = Polygon([(0,0),(0,2),(1,1)])
p2 = Polygon([(2,-1),(2,3),(4,3),(4,-1)])

src = (-1,1)
dst = (5,1)

lines = get_visibility_graph([p1, p2], src, dst)
graph = build_graph(lines)

# for k in graph.keys():
#     print(f"{k}:{graph[k]}")

path = get_shorest_path(lines, src, dst)

print(path)
