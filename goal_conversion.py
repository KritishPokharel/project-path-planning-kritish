from src.graph import GridGraph

g = GridGraph("data/current1.map", collision_radius=15)

x, y = g.cell_to_pos(400, 400)
print(y, x)

"""
maze1.map file format:
origin_x   origin_y   width   height   meters_per_cell

-2.525   -2.525   100   100   0.05
x = (i + 0.5) * meters_per_cell + origin_x
y = (j + 0.5) * meters_per_cell + origin_y
for goal (0, 90):
x = (0 + 0.5) * 0.05 + (-2.525) = -2.5
y = (90 + 0.5) * 0.05 + (-2.525) = 2.0
"""
