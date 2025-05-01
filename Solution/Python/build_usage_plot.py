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
                            map[pos] = float(map[pos]) / STEPS_NUM
                            mn = min(mn, map[pos])
                            mx = max(mx, map[pos])
                        data[-1].append(float(map[pos]))

                result[-1].append(data)
        assert 0 <= mn, "invalid mn: " + str(mn)
        assert mx <= 1, "invalid mx: " + str(mx)
        mx = 1.0
        return result, mn, mx


dirs = ["E", "S", "W", "N", "All"]

acts = ["FW", "R", "CR", "W", "All"]


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


def paint_one(data, mn, mx, to_file, dir, act):
    if map_type == "random":
        figsize = (11, 10)
    elif map_type == "warehouse":
        figsize = (11, 3)
    elif map_type == "game":
        figsize = (11, 10)
    else:
        assert False
    fig, axes = plt.subplots(1, 1, figsize=figsize, constrained_layout=True)
    images = []
    map = np.array(data[dir][act])
    ax = axes

    images.append(ax.imshow(map, cmap=good_cmap, vmin=mn, vmax=mx))

    if True:
        mask = map == -1
        red_mask = np.zeros((*map.shape, 4))
        red_mask[mask] = [0, 0, 0, 1]
        ax.imshow(red_mask)

    ax.axis('off')
    fig.colorbar(images[-1], ax=ax)
    plt.savefig(to_file + ".pdf", format='pdf', dpi=400, bbox_inches='tight')
    plt.close()


def paint_all(data, mn, mx, to_file):
    if map_type == "random":
        figsize = (11, 10)
    elif map_type == "warehouse":
        figsize = (28, 8)
    elif map_type == "game":
        figsize = (36, 30)
    else:
        assert False
    fig, axes = plt.subplots(5, 5, figsize=figsize, constrained_layout=True)

    # Убираем все границы
    for ax in axes.flat:
        for spine in ax.spines.values():
            spine.set_visible(False)

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

    for i in range(5):
        for j in range(5):
            ax = axes[i, j]

            ax.set_xticks([])
            ax.set_yticks([])

            if i == 0:
                ax.set_title(acts[j], pad=10)

            if j == 0:
                ax.set_ylabel(dirs[i], rotation=0, labelpad=10)

    fig.colorbar(images[-1], ax=axes)

    plt.savefig(to_file + ".pdf", format='pdf', dpi=400, bbox_inches='tight')
    plt.close()


if __name__ == '__main__':
    assert len(sys.argv) == 4, "invalid arguments"

    input = sys.argv[1]
    output_name = sys.argv[2]
    map_type = sys.argv[3]

    if map_type == "random":
        STEPS_NUM = 1000
    elif map_type == "warehouse":
        STEPS_NUM = 5000
    elif map_type == "game":
        STEPS_NUM = 5000
    else:
        assert False

    data, mn, mx = read(input)

    last_slash_index = input.rfind('/')
    dir = input[:last_slash_index + 1]

    paint_all(data, mn, mx, dir + output_name + "_full_mesh")
    paint_one(data, mn, mx, dir + output_name + "_one_all", 4, 4)
    paint_one(data, mn, mx, dir + output_name + "_one_wait", 4, 3)
