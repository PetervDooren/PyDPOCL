import sys
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

objcolors = ["red",
            "blue",
            "green",
            "yellow",
            "purple",
            "cyan",
            "orange",
            "white"]

# Load the worldmodel JSON file
def load_worldmodel(filename):
    with open(filename, 'r') as f:
        return json.load(f)

def plot_area(ax, area, color='lightgray', edgecolor='black', alpha=0.5, label=None):
    coords = area['coords']
    poly = MplPolygon(coords, closed=True, facecolor=color, edgecolor=edgecolor, alpha=alpha, label=label)
    ax.add_patch(poly)
    if label:
        # Place label at centroid
        xs, ys = zip(*coords)
        centroid = (sum(xs)/len(xs), sum(ys)/len(ys))
        ax.text(centroid[0], centroid[1], label, ha='center', va='center', fontsize=12)

def plot_object(ax, obj, color='orange', edgecolor='black', alpha=0.9):
    x, y = obj['initial_pose']
    w = obj['width']
    l = obj['length']
    # Rectangle centered at (x, y)
    rect = [
        [x - w/2, y - l/2],
        [x - w/2, y + l/2],
        [x + w/2, y + l/2],
        [x + w/2, y - l/2],
    ]
    poly = MplPolygon(rect, closed=True, facecolor=color, edgecolor=edgecolor, alpha=alpha)
    ax.add_patch(poly)
    ax.text(x, y, obj['name'], ha='center', va='center', fontsize=12, color='black')

def plot_arrow(ax, start_obj, end_area, color='black'):
    x, y = start_obj['initial_pose']

    coords = end_area['coords']
    xs, ys = zip(*coords)
    end_x = sum(xs)/len(xs)
    end_y = sum(ys)/len(ys)
    ax.annotate('', xy=(end_x, end_y), xytext=(x, y),
            arrowprops=dict(arrowstyle='->', color='black', lw=2))

def main():
    filenames = ['domains/manipulation-domain-batch/test_0_worldmodel.json',
                 'domains/manipulation-domain-batch/test_1_worldmodel.json',
                 'domains/manipulation-domain-batch/test_2_worldmodel.json',
                 'domains/manipulation-domain-batch/test_3_worldmodel.json',
                 'domains/manipulation-domain-batch/test_4_worldmodel.json',
                 'domains/manipulation-domain-batch/test_5_worldmodel.json',
                 'domains/manipulation-domain-batch/test_6_worldmodel.json',
                 'domains/manipulation-domain-batch/test_7_worldmodel.json',
                 'domains/manipulation-domain-batch/test_8_worldmodel.json',
                 'domains/manipulation-domain-batch/test_9_worldmodel.json',
                 ]

    num_args = len(sys.argv)
    if num_args > 1:
        filenames = [f'domains/manipulation-domain-batch/test_{i}_worldmodel.json' for i in sys.argv[1:]]

    for filename in filenames:
        data = load_worldmodel(filename)

        fig, ax = plt.subplots(figsize=(8, 6))

        def find_area_by_name(name):
            for area in data['areas']:
                if area['name'] == name:
                    return area
            return None
        
        def find_object_by_name(name):
            for obj in data['objects']:
                if obj['name'] == name:
                    return obj
            return None

        # Plot areas
        plot_area(ax, find_area_by_name('table'))
        plot_area(ax, find_area_by_name('reach_robot_0'))
        plot_area(ax, find_area_by_name('reach_robot_1'))

        # Plot goal regions
        i = 0
        while True:
            goal_name = f'goal_{i}'
            goal_area = find_area_by_name(goal_name)
            if goal_area is None:
                break
            obj_color = objcolors[i]
            plot_area(ax, goal_area, color=obj_color, edgecolor='red', label=goal_name)
            i += 1
        
        # Plot objects
        i = 0
        while True:
            obj_name = f'box_{i}'
            goal_name = f'goal_{i}'
            obj = find_object_by_name(obj_name)
            goal_area = find_area_by_name(goal_name)
            if obj is None:
                break
            obj_color = objcolors[i] if goal_area is not None else 'gray'
            plot_object(ax, obj, color=obj_color)
            if goal_area is not None:
                #plot_arrow(ax, obj, goal_area)
                pass
            i += 1
        
        #for area in data['areas']:
        #    if area['name'] == 'table': 
        #        plot_area(ax, area, color='lightgrey', label=None)
        #    elif area['name'] == 'reach_robot_0':
        #        plot_area(ax, area, color='lightgrey', label=None)
        #    elif area['name'] == 'reach_robot_1':
        #        plot_area(ax, area, color='lightgrey', label=None)
        #    else:
        #        plot_area(ax, area, color='blue', label=area['name'])

        # Plot objects
        #for obj in data['objects']:
        #    plot_object(ax, obj)

        ax.set_aspect('equal')
        ax.autoscale()
        ax.set_title(f'Worldmodel Visualization: {filename}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

if __name__ == '__main__':
    main()
