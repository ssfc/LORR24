import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.colors
import seaborn as sns
import pandas as pd
from PIL import Image

plan_algos = [
    "pepibt(4)_lns+gg+gs", "pepibt(4)_lns+gs",
    "epibt(4)+gg+gs", "epibt(4)+gs",

    "pibt+gg+gs", "pibt+gs",
    "wppl+gg+gs",
    "pibt_tf+gs",
]
# colors = ['lime', 'dodgerblue', 'orange', 'red', 'blueviolet', 'aqua', 'deeppink', 'brown']
# colors = ['green', 'blue', 'orange', 'red', 'blueviolet', 'aqua', 'deeppink', 'brown']
plan_algos_name = ['EPIBT+LNS+GG', 'EPIBT+LNS', 'EPIBT+GG', 'EPIBT', 'PIBT+GG', 'PIBT', 'WPPL+GG', 'PIBT+traffic flow']
markers = ['o', 'v', 's', 'p', '*', 'x', 'D', 'P', 'o', 'v', 's', 'p', '*', 'x', 'D', 'P', 'o', 'v', 's', 'p', '*', 'x', 'D', 'P']

color_palette = sns.color_palette("tab10", 8)
plt.rcParams['axes.prop_cycle'] = plt.cycler(color=color_palette)


def add_map(map_name, map_text, column):
    df = pd.read_csv('../../NewData/' + map_name + '/total_metrics.csv', sep=';')
    df['throughput'] = df['throughput'].astype(float)
    grouped = df.groupby('algo name')

    ax = axes[0][column]
    ax.imshow(np.asarray(Image.open(map_name + '.png')))
    ax.title.set_text(map_text)
    ax.set_xticks([])
    ax.set_yticks([])

    ax = axes[1][column]
    for i in range(len(plan_algos)):
        try:
            df = grouped.get_group(plan_algos[i])
            ax.plot(df['agents num'], df['throughput'], alpha=1, label=plan_algos_name[i], marker=markers[i])  # , color=colors[i])
            if map_name == "random":
                ax.set_ylabel('Throughput')
            ax.grid(True)
        except:
            print("no group:", plan_algos[i])

    ax = axes[2][column]
    for i in range(len(plan_algos)):
        try:
            df = grouped.get_group(plan_algos[i])
            ax.plot(df['agents num'], df['avg step time'], alpha=1, label=plan_algos_name[i],
                    marker=markers[i])  # , color=colors[i])
            ax.set_yscale('log')
            if map_name == "random":
                ax.set_ylabel('Decision Time (ms)')
            ax.grid(True)
            ax.set_xlabel('Number of Agents')
        except:
            print("no group:", plan_algos[i])


if __name__ == '__main__':
    fig, axes = plt.subplots(3, 3, figsize=(10, 10))

    add_map('random', 'random-32-32-20\nSize: 32x32\n|V|=819', 0)
    add_map('warehouse', 'warehouse\nSize: 140x500\n|V|=38586', 1)
    add_map('game', 'brc202d\nSize: 530x481\n|V|=43151', 2)

    lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    # remove not unique lines
    while True:
        kek = labels.copy()
        kek.pop(-1)
        if labels[-1] in kek:
            lines.pop(-1)
            labels.pop(-1)
        else:
            break
    print(labels)
    fig.legend(lines, labels, loc='lower center', ncol=4)

    plt.savefig("metrics_plot.pdf", format='pdf', dpi=800)
    plt.show()
