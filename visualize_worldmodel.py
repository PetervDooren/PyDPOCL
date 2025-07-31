import sys
import json
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

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
        ax.text(centroid[0], centroid[1], label, ha='center', va='center', fontsize=8)

def plot_object(ax, obj, color='orange', edgecolor='red', alpha=0.7):
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
    ax.text(x, y, obj['name'], ha='center', va='center', fontsize=8, color='black')

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
        filenames = [filenames[int(i)] for i in sys.argv[1:]]

    for filename in filenames:
        data = load_worldmodel(filename)

        fig, ax = plt.subplots(figsize=(8, 6))

        # Plot areas
        for area in data['areas']:
            plot_area(ax, area, label=area['name'])

        # Plot objects
        for obj in data['objects']:
            plot_object(ax, obj)

        ax.set_aspect('equal')
        ax.autoscale()
        ax.set_title(f'Worldmodel Visualization: {filename}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

if __name__ == '__main__':
    main()
