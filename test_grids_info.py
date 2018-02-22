from grids_info import GridsInfo
from sortedcontainers import SortedListWithKey

grids_info = GridsInfo(100, 100)

grids_info.set_g_cost(0, 0, 12221.3)

grids_info.set_g_cost(1, 1, 10.0)
grids_info.set_h_cost(1, 1, 10.0)

open_list = SortedListWithKey(key=lambda t: grids_info.get_f_cost(t[0], t[1]))

open_list.add((0,0))
open_list.add((1,1))

if (2, 2) in open_list:
    open_list.remove((2, 2))
grids_info.set_g_cost(2, 2, 1.0)
grids_info.set_h_cost(2, 2, 1.0)
print(len(open_list))
open_list.add((2, 2))

print(len(open_list))
print(open_list.pop(0))
print(len(open_list))

print('Finished!')