import math

from matplotlib import animation
import matplotlib.pyplot as plt
import csv
import numpy as np
from matplotlib.patches import Rectangle
import networkx as nx
import pydot
from networkx.drawing.nx_pydot import graphviz_layout ,write_dot
from numpy import pi
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import matplotlib.path as mpath
from matplotlib.collections import PatchCollection
from pint import UnitRegistry

sub_folder = ''



time_steps = 600
id = '36'


original_SwarmX = []
with open(sub_folder+'Original_SwarmX'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        original_SwarmX.append([val for val in row])

original_SwarmY = []
with open(sub_folder+'Original_SwarmY'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        original_SwarmY.append([val for val in row])


imitated_SwarmX = []
with open(sub_folder+'imitated_SwarmX'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        imitated_SwarmX.append([val for val in row])


imitated_SwarmY = []
with open(sub_folder+'imitated_SwarmY'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        imitated_SwarmY.append([val for val in row])


original_BT = []
with open(sub_folder+'Original_BT'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        original_BT.append([val for val in row])

del original_BT[0][len(original_BT[0])-1]
del original_BT[1][len(original_BT[1])-1]

original_Tree_Nodes=list(map(lambda x: int(x.replace(",", "")), original_BT[0]))
original_Tree_ChildNum=list(map(lambda x: int(x.replace(",", "")), original_BT[1]))

imitated_BT = []
with open(sub_folder+'imitated_BT'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        imitated_BT.append([val for val in row])

del imitated_BT[0][len(imitated_BT[0])-1]
del imitated_BT[1][len(imitated_BT[1])-1]

imitated_Tree_Nodes=list(map(lambda x: int(x.replace(",", "")), imitated_BT[0]))
imitated_Tree_ChildNum=list(map(lambda x: int(x.replace(",", "")), imitated_BT[1]))

original_Env = []
with open(sub_folder+'Original_SwarmEnv'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        original_Env.append([val for val in row])

if (int(original_Env[0][0])==1):
    original_AreasX = []
    with open(sub_folder+'Original_SwarmEnvAreasX'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            original_AreasX.append([val for val in row])


    original_AreasY = []
    with open(ub_folder+'Original_SwarmEnvAreasY'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            original_AreasY.append([val for val in row])


    original_AreasW = []
    with open(sub_folder+'Original_SwarmEnvAreasWidth'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            original_AreasW.append([val for val in row])

    del original_AreasX[0][len(original_AreasX[0])-1]
    del original_AreasY[0][len(original_AreasY[0])-1]
    del original_AreasW[0][len(original_AreasW[0])-1]

if (int(original_Env[0][2])==1):
    originalObstaclesX = []
    with open(sub_folder+'Original_SwarmEnvObstaclesX'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            originalObstaclesX.append([val for val in row])


    originalObstaclesY = []
    with open(sub_folder+'Original_SwarmEnvObstaclesY'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            originalObstaclesY.append([val for val in row])

    originalObstaclesX2 = []
    with open(sub_folder+'Original_SwarmEnvObstaclesX2'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            originalObstaclesX2.append([val for val in row])


    originalObstaclesY2 = []
    with open(sub_folder+'Original_SwarmEnvObstaclesY2'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            originalObstaclesY2.append([val for val in row])


    del originalObstaclesX[0][len(originalObstaclesX[0])-1]
    del originalObstaclesY[0][len(originalObstaclesY[0])-1]
    del originalObstaclesX2[0][len(originalObstaclesX2[0])-1]
    del originalObstaclesY2[0][len(originalObstaclesY2[0])-1]

if (int(original_Env[0][1])==1):
    originalObjectsX = []
    with open(sub_folder+'Original_SwarmEnvObjectX'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            originalObjectsX.append([val for val in row])


    originalObjectsY = []
    with open(sub_folder+'Original_SwarmEnvObjectY'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            originalObjectsY.append([val for val in row])

    for row in originalObjectsX:
        del row[len(row)-1]

    for row in originalObjectsY:
        del row[len(row)-1]


imitated_Env = []
with open(sub_folder+'imitated_SwarmEnv'+id+'.csv', 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=",", quotechar='|')
    for row in reader:
        imitated_Env.append([val for val in row])

if (int(imitated_Env[0][0])==1):
    imitated_AreasX = []
    with open(sub_folder+'imitated_SwarmEnvAreasX'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_AreasX.append([val for val in row])


    imitated_AreasY = []
    with open(sub_folder+'imitated_SwarmEnvAreasY'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_AreasY.append([val for val in row])


    imitated_AreasW = []
    with open(sub_folder+'imitated_SwarmEnvAreasWidth'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_AreasW.append([val for val in row])

    del imitated_AreasX[0][len(imitated_AreasX[0])-1]
    del imitated_AreasY[0][len(imitated_AreasY[0])-1]
    del imitated_AreasW[0][len(imitated_AreasW[0])-1]

if (int(imitated_Env[0][2])==1):
    imitated_ObstaclesX = []
    with open(sub_folder+'imitated_SwarmEnvObstaclesX'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_ObstaclesX.append([val for val in row])

    imitated_ObstaclesY = []
    with open(sub_folder+'imitated_SwarmEnvObstaclesY'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_ObstaclesY.append([val for val in row])

    imitated_ObstaclesX2 = []
    with open(sub_folder+'imitated_SwarmEnvObstaclesX2'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_ObstaclesX2.append([val for val in row])

    imitated_ObstaclesY2 = []
    with open(sub_folder+'imitated_SwarmEnvObstaclesY2'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_ObstaclesY2.append([val for val in row])

    del imitated_ObstaclesX[0][len(imitated_ObstaclesX[0])-1]
    del imitated_ObstaclesY[0][len(imitated_ObstaclesY[0])-1]
    del imitated_ObstaclesX2[0][len(imitated_ObstaclesX2[0])-1]
    del imitated_ObstaclesY2[0][len(imitated_ObstaclesY2[0])-1]

if (int(imitated_Env[0][1])==1):
    imitated_ObjectsX = []
    with open(sub_folder+'imitated_SwarmEnvObjectX'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_ObjectsX.append([val for val in row])

    imitated_ObjectsY = []
    with open(sub_folder+'imitated_SwarmEnvObjectY'+id+'.csv', 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=",", quotechar='|')
        for row in reader:
            imitated_ObjectsY.append([val for val in row])

    for row in imitated_ObjectsX:
        del row[len(row)-1]

    for row in imitated_ObjectsY:
        del row[len(row)-1]

Tree_Nodes= []
Tree_ChildNum= []

edges = []
leaf_Nodes = []

original_trails_x=[]
original_trails_y=[]
imitated_trails_x=[]
imitated_trails_y=[]

def tree_viz2 (tree,leaf_Nodes,index):
    Node = tree[index]
    if Node ==2 :
        leaf_Nodes.append("Repulsion" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 3:
        leaf_Nodes.append("aggregation" +  str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 4:
        leaf_Nodes.append("NW_force" +  str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 5:
        leaf_Nodes.append("NE_force" +  str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 6:
        leaf_Nodes.append("SE_force" +  str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 7:
        leaf_Nodes.append("SW_force" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 8:
        leaf_Nodes.append("Random" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 9:
        leaf_Nodes.append("Boundary Force" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 10:
        leaf_Nodes.append("neighbor in area?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 11:
        leaf_Nodes.append("In area>10s?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 12:
        leaf_Nodes.append("t>30?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 13:
        leaf_Nodes.append("No Neighbors?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 14:
        leaf_Nodes.append("neighbor picked object?" + str(index))
        index-=1
        return tree_viz2(tree, leaf_Nodes, index)
    elif Node == 15:
        leaf_Nodes.append("Out of Boundary?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 16:
        leaf_Nodes.append("Inside Area?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 17:
        leaf_Nodes.append("Sensed Area?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 18:
        leaf_Nodes.append("Pick obj" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 19:
        leaf_Nodes.append("Drop obj" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 20:
        leaf_Nodes.append("Obstacle Avoidance" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 21:
        leaf_Nodes.append("Send Msg" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 22:
        leaf_Nodes.append("Received Msg?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 23:
        leaf_Nodes.append("Picked object?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 24:
        leaf_Nodes.append("Picked object>10?" + str(index))
        index-=1
        return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 1:
        for i in range(Tree_ChildNum[index]):
            edges.append(("Sel" + str(index), leaf_Nodes.pop()))
        leaf_Nodes.append("Sel" + str(index))
        if (index <=0):
            graph = nx.DiGraph()
            graph.add_edges_from(reversed(edges))
            root="Sel"+str(index)
            treeDFS = nx.dfs_edges(graph, root)
            graph2 = nx.MultiDiGraph()
            graph2.add_edges_from(treeDFS)
            return graph2
        else:
            index -= 1
            return tree_viz2(tree,leaf_Nodes,index)
    elif Node == 0:
        for i in range(Tree_ChildNum[index]):
            edges.append(("Seq" + str(index), leaf_Nodes.pop()))
        leaf_Nodes.append("Seq" + str(index))
        if (index <= 0):
            graph = nx.DiGraph()
            graph.add_edges_from(reversed(edges))
            root = "Seq" + str(index)
            treeDFS = nx.dfs_edges(graph, root)
            graph2 = nx.MultiDiGraph()
            graph2.add_edges_from(treeDFS)
            return graph2
        else:
            index -= 1
            return tree_viz2(tree, leaf_Nodes, index)



def animate(i):

    x=list(map(lambda x: float(x.replace(",", "")), original_SwarmX[i]))
    y=list(map(lambda x: float(x.replace(",", "")), original_SwarmY[i]))

    x1=list(map(lambda x: float(x.replace(",", "")), imitated_SwarmX[i]))
    y1=list(map(lambda x: float(x.replace(",", "")), imitated_SwarmY[i]))


    if (int(original_Env[0][1])==1):
        x2=list(map(lambda x: float(x.replace(",", "")), originalObjectsX[i]))
        y2=list(map(lambda x: float(x.replace(",", "")), originalObjectsY[i]))

        OriginalObjects.set_data(x2, y2)
    else:
        OriginalObjects.set_data([], [])

    if (int(imitated_Env[0][1])==1):
        x3=list(map(lambda x: float(x.replace(",", "")), imitated_ObjectsX[i]))
        y3=list(map(lambda x: float(x.replace(",", "")), imitated_ObjectsY[i]))

        ImitatedObjects.set_data(x3, y3)
    else:
        ImitatedObjects.set_data([], [])

    label1.set_text('Time: (%d)' % (i))
    OriginalRadius.set_data(x, y)
    OriginalSwarm.set_data(x, y)
 
    label2.set_text('Time: (%d)' % (i))
    ImitatedRadius.set_data(x1, y1)
    ImitatedSwarm.set_data(x1, y1)
 

    return (label1,label2,OriginalSwarm,OriginalRadius,OriginalObjects,ImitatedSwarm,ImitatedRadius,ImitatedObjects,
            Original_trails_1,Original_trails_2,Original_trails_3,Original_trails_4,)



def init():
    # ---> Adding a print statement here <---
    print('Initializing')
    OriginalSwarm.set_data([], [])
    OriginalRadius.set_data([], [])
    OriginalObjects.set_data([], [])
    ImitatedSwarm.set_data([], [])
    ImitatedRadius.set_data([], [])
    ImitatedObjects.set_data([], [])
    Original_trails_1.set_data([], [])
    Original_trails_2.set_data([], [])
    Original_trails_3.set_data([], [])
    Original_trails_4.set_data([], [])
    label1.set_text('')
    label2.set_text('')

    return (label1,label2,OriginalSwarm,OriginalRadius,OriginalObjects,ImitatedSwarm,ImitatedRadius,ImitatedObjects,
            Original_trails_1,Original_trails_2,Original_trails_3,Original_trails_4,)

# Initialize function for the trails


fig = plt.figure(figsize=(14,14), dpi=100, facecolor='w', edgecolor='k')

ax1 = fig.add_subplot(221)
ax1.title.set_text('Original Swarm Behavior')
label1 = ax1.text(280, 320, " ", ha='center', va='center', fontsize=12, color="purple")
time_text = ax1.text(-20, 26, '', fontsize=12)
box_text = ax1.text(3, 26, '', color='purple', fontsize=12)


robot_radius= 3
rs= (robot_radius*ax1.transData.transform((1, 1))[0])/500
robot_area = (pi*(rs-1))
#sensing_radius= robot_radius+ 15 # works perfectly for simulation radius
sensing_radius= robot_radius+ 5.5# 12.6 works perfectly for simulation radius
ms= (sensing_radius*ax1.transData.transform((1, 1))[0])/500
sensing_area = (pi*(ms-1))
object_width= 3
os= (object_width*ax1.transData.transform((1, 1))[0])/500
object_area = os*os

sensing_area = int(fig.dpi * 2 * sensing_radius * fig.get_figwidth()
         / np.diff(ax1.get_xbound())[0])

OriginalRadius, =ax1.plot([], [], 'o', markeredgecolor="m",markersize=32,fillstyle='none', alpha=0.07)
box_radius = 2
OriginalSwarm, = ax1.plot([], [], 'o', color='purple', markersize=8, markeredgecolor="black", alpha=0.8)
#OriginalMode, = ax1.plot([], [], '*', color='b', markersize=robot_area, markeredgecolor="black", alpha=0.9)
OriginalObjects, = ax1.plot([], [], 'bs', markersize=12, markeredgecolor="black", alpha=0.5)
#Original_trail_lines = [ax1.plot([], [], color='blue', alpha=0.3, linewidth=1)[0] for _ in range(num_agents)]
#Original_trail_dots = [[ax1.scatter([], [], color='blue', alpha=1.0, s=10) for _ in range(trail_length)] for _ in range(num_agents)]
Original_trails_1, = ax1.plot([], [], 'o', color='purple', markersize=8, markeredgecolor="black", alpha=0.2)
Original_trails_2, = ax1.plot([], [], 'o', color='purple', markersize=8, markeredgecolor="black", alpha=0.1)
Original_trails_3, = ax1.plot([], [], 'o', color='purple', markersize=8, markeredgecolor="black", alpha=0.05)
Original_trails_4, = ax1.plot([], [], 'o', color='purple', markersize=8, markeredgecolor="black", alpha=0.01)
if (int(original_Env[0][0])==1):
    for i in range(len(original_AreasX[0])):
        #area_center_X = float(original_AreasX[0][i]) + (float(original_AreasW[0][i])/2)
        #area_center_Y = float(original_AreasY[0][i]) + (float(original_AreasW[0][i])/2)
        #ax1.plot(area_center_X ,area_center_Y ,'*')
        #ax1.plot(area_center_X + (float(original_AreasW[0][i])/(2)),area_center_Y + (float(original_AreasW[0][i])/(2)),'*')
        ax1.add_patch(Rectangle((float(original_AreasX[0][i]), float(original_AreasY[0][i])), float(original_AreasW[0][i]), float(original_AreasW[0][i]),fc="green",alpha=0.2,ec="green"))
        #circle1 = plt.Circle((area_center_X, area_center_Y), float(original_AreasW[0][i])/2, color='r',alpha=0.1)
        #circle2 = plt.Circle((area_center_X+ float(original_AreasW[0][i])/(2), area_center_Y+ float(original_AreasW[0][i])/(2)), 5, color='r',alpha=0.1)

        #circle2 = plt.Circle((float(original_AreasX[0][i]) + (float(original_AreasW[0][i])/(5)), float(original_AreasY[0][i]) + (float(original_AreasW[0][i])/(5))), float(original_AreasW[0][i])/(5), color='r',alpha=0.1)
        #circle3 = plt.Circle((float(original_AreasX[0][i]) + (float(original_AreasW[0][i])/(5)), float(original_AreasY[0][i]) + (float(original_AreasW[0][i])/(5))+(float(original_AreasW[0][i])/(2))), float(original_AreasW[0][i])/(5), color='r',alpha=0.1)
        #ax1.add_patch(circle1)
        #ax1.add_patch(circle2)
        #ax1.add_patch(circle3)

if (int(original_Env[0][2])==1):
    for i in range(len(originalObstaclesX[0])):
        ax1.plot([float(originalObstaclesX[0][i]),float(originalObstaclesX2[0][i])],[float(originalObstaclesY[0][i]),float(originalObstaclesY2[0][i])],color="black")
        #l = Line2D([0, 1], [0, 1], color="green")
        #ax1.add_artist(l)


dim = 350
ax1.set_xlim((-dim, dim))
ax1.set_ylim((-dim, dim))
ax1.add_patch(Rectangle ((-250, -250), 500, 500,fc="white",alpha=0.4,ec="black"))

ax2 = fig.add_subplot(222)
ax2.title.set_text('Imitated Swarm Behavior')
ImitatedSwarm, = ax2.plot([], [], 'o', color='g', markersize=8, markeredgecolor="black", alpha=0.9)
label2 = ax2.text(280, 320, " ", ha='center', va='center', fontsize=12, color="green")
time_text2 = ax2.text(-20, 26, '', fontsize=12)
box_text2 = ax2.text(3, 26, '', color='red', fontsize=12)
ImitatedRadius, =ax2.plot([], [], 'o', markeredgecolor="m",markersize=30,fillstyle='none', alpha=0.07)
ImitatedObjects, = ax2.plot([], [], 'bs', markersize=12, markeredgecolor="black", alpha=0.5)

if (int(imitated_Env[0][0])==1):
    for i in range(len(imitated_AreasX[0])):
        ax2.add_patch(Rectangle((float(imitated_AreasX[0][i]), float(imitated_AreasY[0][i])), float(imitated_AreasW[0][i]), float(imitated_AreasW[0][i]),fc="green",alpha=0.2,ec="green"))

if (int(imitated_Env[0][2])==1):
    for i in range(len(imitated_ObstaclesX[0])):
        ax2.plot([float(imitated_ObstaclesX[0][i]),float(imitated_ObstaclesX2[0][i])],[float(imitated_ObstaclesY[0][i]),float(imitated_ObstaclesY2[0][i])],color="black")
        #l = Line2D([0, 1], [0, 1], color="green")
        #ax1.add_artist(l)

ax2.set_xlim((-dim, dim))
ax2.set_ylim((-dim, dim))
ax2.add_patch(Rectangle ((-250, -250), 500, 500,fc="white",alpha=0.4,ec="black"))


Tree_Nodes= original_Tree_Nodes
Tree_ChildNum= original_Tree_ChildNum
edges = []
leaf_Nodes =[]
graph1 = tree_viz2(Tree_Nodes,leaf_Nodes,len(Tree_Nodes)-1)
pos1=graphviz_layout(graph1, prog='dot')
ax3 = fig.add_subplot(223)
ax3.title.set_text('Original Behavior Tree')

ax3.text(0,0,(nx.draw(graph1, pos=pos1,
                      node_color='red',
                      node_size=1200,
                      with_labels=True,
                      font_size=10,
                      arrows=True)))



Tree_Nodes= imitated_Tree_Nodes
Tree_ChildNum= imitated_Tree_ChildNum
edges = []
leaf_Nodes =[]
graph2 = tree_viz2(Tree_Nodes,leaf_Nodes,len(Tree_Nodes)-1)

pos2=graphviz_layout(graph2, prog='dot')

ax4 = fig.add_subplot(224)
ax4.title.set_text('Extracted Behavior Tree')

ax4.text(0,0,(nx.draw(graph2, pos=pos2,
                      node_color='lightgreen',
                      node_size=1200,
                      with_labels=True,
                      font_size = 10,
                      arrows=True)))




fig.tight_layout()

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=time_steps, interval=100, blit=True)

anim
plt.show()


#FFwriter = animation.FFMpegWriter(fps=10, extra_args=['-vcodec', 'libx264'])
#anim.save('Swarm1.mp4', writer=FFwriter)



