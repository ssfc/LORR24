import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.colors


def read(filename):
    with open(filename) as f:
        lines = [line.rstrip('\n') for line in f]
        rows, cols = lines[0].split(' ')
        rows = int(rows)
        cols = int(cols)
        # print(rows, cols)
        result = []
        mn = 1000000000
        mx = 0
        for dir in range(0, 5):
            result.append([])
            for act in range(0, 5):
                map = [int(x) for x in lines[1 + dir * 6 + act].split(' ')]
                # print(map)

                data = []
                for x in range(0, rows):
                    data.append([])
                    for y in range(0, cols):
                        pos = x * cols + y + 1
                        if map[pos] != -1:
                            map[pos] = float(map[pos]) / 5000
                            mn = min(mn, map[pos])
                            mx = max(mx, map[pos])
                        data[-1].append(float(map[pos]))

                result[-1].append(data)
        assert 0 <= mn, "invalid mn: " + str(mn)
        assert mx <= 1, "invalid mx: " + str(mx)
        mx = 1.0
        return result, mn, mx


dirs = ["E", "S", "W", "N", "A"]

acts = ["FW", "R", "CR", "W", "A"]


def build(directory):
    for id in range(6):
        print(id)
        data, mn, mx = read(directory + "meta" + str(id))

        fig, axes = plt.subplots(5, 5, figsize=(10, 10))
        images = []
        for i in range(25):
            dir = i // 5
            act = i % 5
            map = data[dir][act]
            ax = axes[dir][act]
            # print(dir, act, map)
            images.append(ax.imshow(map, cmap='viridis'))  # , vmin=mn, vmax=mx))
            ax.set_title(dirs[dir] + " & " + acts[act])
            ax.axis('off')
            fig.colorbar(images[-1], ax=ax)

        # plt.plot(np.where(map == -500, map, None), color="red", label="1")

        # fig.colorbar(images[-1], ax=axes.ravel().tolist())
        plt.savefig(directory + "usage" + str(id) + ".svg", format='svg', dpi=1200)


def build2(directory):
    data, mn, mx = read(directory + "meta")

    for i in range(25):
        fig, axes = plt.subplots(1, 1, figsize=(10, 10))
        dir = i // 5
        act = i % 5
        map = data[dir][act]
        ax = axes
        print("processing:", dir, act)
        ax.imshow(map, cmap='viridis')
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')
        plt.tight_layout()
        # plt.show()
        plt.savefig(dirs[dir] + "_" + acts[act] + ".svg", format='svg', dpi=1200)


good_cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", [(0, '#008064'), (.1, "#FFFF64"), (0.5, "#FF6464"),
                                                                     (1, "#960064")])


def paint_one(data, mn, mx, to_file):
    fig, axes = plt.subplots(1, 1, figsize=(6, 2))
    images = []
    dir = 4
    act = 4
    map = np.array(data[dir][act])  # матрица
    ax = axes

    images.append(ax.imshow(map, cmap=good_cmap, vmin=mn, vmax=mx))

    if True:
        mask = map == -1
        red_mask = np.zeros((*map.shape, 4))
        red_mask[mask] = [0, 0, 0, 1]
        ax.imshow(red_mask)

    # ax.set_title(dirs[dir] + " & " + acts[act])
    ax.axis('off')

    # fig.colorbar(images[-1], ax=ax)

    plt.savefig(to_file, format='pdf', dpi=800)
    # plt.show()


def paint_all(data, mn, mx, to_file):
    fig, axes = plt.subplots(5, 5, figsize=(10, 10))
    images = []
    for i in range(25):
        dir = i // 5
        act = i % 5
        map = np.array(data[dir][act])  # матрица
        ax = axes[dir][act]

        images.append(ax.imshow(map, cmap=good_cmap, vmin=mn, vmax=mx))

        if True:
            mask = map == -1
            red_mask = np.zeros((*map.shape, 4))
            red_mask[mask] = [0, 0, 0, 1]
            ax.imshow(red_mask)

        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')

    fig.colorbar(images[-1], ax=axes)

    plt.savefig(to_file, format='pdf', dpi=800)
    # plt.show()


if __name__ == '__main__':
    begin_path = "../../NewData/game/"
    end_path = "/usage2.txt"

    # warehouse
    '''algos = [
        ["pepibt(4)_lns+gg+gs", "EPIBT+LNS+GG"],
        ["pepibt(4)_lns+gs", "EPIBT+LNS"],

        ["epibt(4)+gg+gs", "EPIBT+GG"],
        ["epibt(4)+gs", "EPIBT"],

        ["pibt+gg+gs", "PIBT+GG"],
        ["pibt+gs", "PIBT"],

        ["wppl+gg+gs", "WPPL+GG"],
        ["pibt_tf+gs", "PIBT+traffic flow"],
    ]'''

    # game
    algos = [
        ["pepibt(4)_lns+gg+gs", "EPIBT+LNS+GG"],
        ["epibt(4)+gg+gs", "EPIBT+GG"],
        ["pibt+gg+gs", "PIBT+GG"],
        ["wppl+gg+gs", "WPPL+GG"],

        ["pepibt(4)_lns+gs", "EPIBT+LNS"],
        ["epibt(4)+gs", "EPIBT"],
        ["pibt+gs", "PIBT"],
        ["pibt_tf+gs", "PIBT+traffic flow"],
    ]

    to_file = "game_plots.pdf"

    fig, axes = plt.subplots(2, 4, figsize=(12, 5), constrained_layout=True)
    #fig, axes = plt.subplots(4, 2, figsize=(10, 6), constrained_layout=True)

    row = 0
    col = 0
    images = []
    for [algo, title] in algos:
        data, mn, mx = read(begin_path + algo + end_path)
        dir = 4
        act = 3
        map = np.array(data[dir][act])  # матрица
        ax = axes[row][col]
        col += 1
        if col == 4:
            col = 0
            row += 1

        images.append(ax.imshow(map, cmap=good_cmap, vmin=mn, vmax=mx))

        if True:
            mask = map == -1
            red_mask = np.zeros((*map.shape, 4))
            red_mask[mask] = [0, 0, 0, 1]
            ax.imshow(red_mask)

        ax.set_title(title)
        ax.axis('off')

    fig.colorbar(images[-1], ax=axes)
    plt.savefig(to_file, format='pdf', dpi=800)
