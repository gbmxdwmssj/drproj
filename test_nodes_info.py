from nodes_info import NodesInfo
from sortedcontainers import SortedListWithKey

nodes_info = NodesInfo(100, 100)

nodes_info.set_g_cost(0, 0, 12221.3)

nodes_info.set_g_cost(1, 1, 10.0)
nodes_info.set_h_cost(1, 1, 10.0)

open_list = SortedListWithKey(key=lambda t: nodes_info.get_f_cost(t[0], t[1]))

open_list.add((0,0))
open_list.add((1,1))

if (2, 2) in open_list:
    open_list.remove((2, 2))
nodes_info.set_g_cost(2, 2, 1.0)
nodes_info.set_h_cost(2, 2, 1.0)
print(len(open_list))
open_list.add((2, 2))

print(len(open_list))
print(open_list.pop(0))
print(len(open_list))

print('Finished!')