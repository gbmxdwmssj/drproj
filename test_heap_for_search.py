from heap_for_search import HeapForSearch
from node_for_search import NodeForSearch

open_list = HeapForSearch(key=lambda x: x.get_f_cost())

node1 = NodeForSearch(0.6, 0.7, 0, 0)
node2 = NodeForSearch(0.05, 0.05, 2, 2)
node3 = NodeForSearch(6.0, 7.0, 1, 1)

print(node1.successor_grids)

open_list.add(node3)
open_list.add(node2)
open_list.add(node1)

open_list.pop(0).print()
open_list.pop(0).print()
open_list.pop(0).print()

print(open_list.has(2, 2))

print('Finished!')