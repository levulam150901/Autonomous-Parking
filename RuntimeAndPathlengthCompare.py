from PathPlanningRunner import *;
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def runtimeAndPathLengthCalculate(algorithm_function, start, goal, obstacle_list, rand_area_x, rand_area_y):
    start_time_ns = time.time_ns()

    if (rand_area_x != 0 and rand_area_y != 0):
        rx_m, ry_m, path_length = algorithm_function(start, goal, obstacle_list, rand_area_x, rand_area_y)
    else:
        rx, ry, ryaw, path_length = algorithm_function(start, goal, obstacle_list)

    end_time_ns = time.time_ns()
    execution_time = (end_time_ns - start_time_ns) / 1000000.0

    return execution_time, path_length


def drawCompareChart(data_array):
    names, runtimes, lengths = zip(*data_array)

    lengths_list = list(lengths)

    for i in range(len(lengths_list)):
        if not isinstance(lengths_list[i], (int, float)):
            lengths_list[i] = float(lengths_list[i])

    bar_width = 0.35
    bar_positions_1 = np.arange(len(names))
    bar_positions_2 = bar_positions_1 + bar_width

    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Plot runtime on the first y-axis
    bars1 = ax1.bar(bar_positions_1, runtimes, bar_width, label='Runtime (ms)', color='tab:red')
    ax1.set_xlabel('Algorithms')
    ax1.set_ylabel('Runtime (ms)', color='tab:red')
    ax1.tick_params(axis='y', labelcolor='tab:red')

    # Create a second y-axis for length
    ax2 = ax1.twinx()
    bars2 = ax2.bar(bar_positions_2, lengths_list, bar_width, label='Length (cm)', color='tab:blue')
    ax2.set_ylabel('Path Length (m)', color='tab:blue')
    ax2.tick_params(axis='y', labelcolor='tab:blue')

    for bar, value in zip(bars1, runtimes):
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() / 30,
                 f'{value:.2f}', ha='center', va='bottom', color='tab:red')

    for bar, value in zip(bars2, lengths_list):
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                 f'{value:.2f}', ha='center', va='bottom', color='tab:blue')

    ax1.set_ylim(0, max(runtimes) * 1.2)
    ax2.set_ylim(0, max(lengths_list) * 1.2)

    legend_elements = [
        mpatches.Patch(color='tab:red', label='Runtime (ms)'),
        mpatches.Patch(color='tab:blue', label='Path Length (m)'),
    ]

    ax2.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.0, 1.0), frameon=False)

    fig.tight_layout()
    # plt.subplots_adjust(bottom=0.1)

    plt.title('Runtime and Path Length for each Algorithm')
    plt.xticks(bar_positions_1 + bar_width / 2, names)
    plt.axis('auto')
    plt.show()


if __name__ == '__main__':
    # Define the start and goal positions
    start = (-1.85, 0.1325, -np.pi / 2)  # Start Position [x,y, start dir in rad]
    goal = (-1.00, -0.50, 0.0)  # Goal Position [x,y, goal dir in rad]

    # Define search area
    rand_area_x_m = [-2.5, 2.5]
    rand_area_y_m = [-2.5, 2.5]

    # Define obstacle list [x position, y position]
    obstacle_list_1 = [
        [-1.3749988998291784, 1.375000128910268, 0.25],
        [-1.3750000884965963, 1.625000432029795, 0.25],
        [-1.375000230425759, 1.875000805427043, 0.25],
        [-1.3750039862386922, 2.125002489301276, 0.25],
        [-1.375007027990223, 1.1249963851911506, 0.25],
        [-1.1250034221984053, 1.1250076953832377, 0.25],
        [-0.8749892472055414, 1.125015403398453, 0.25],
        [-0.8750000670440002, 0.8750106288420382, 0.25],
        [-0.8750022838851056, 0.6250103154977967, 0.25],
        [-0.8749885064872707, 0.3750001428132739, 0.25],
        [-0.8749978362024249, 0.12499923812843783, 0.25],
        [-0.8749987578926902, -0.12501242744493973, 0.25],
        [-1.1250026391592498, -0.12500134394135037, 0.25],
        [-1.3750037733287015, -0.12499925336079447, 0.25],
        [-0.8749998576763511, -0.8749992918358481, 0.25],
        [-1.1250008202302646, -0.8749984416690622, 0.25],
        [-1.3750013672761283, -0.8750003131859605, 0.25],
        [-0.624991282832163, -0.8750109533594129, 0.25],
        [-0.6249961585746708, -0.6250102520458617, 0.25],
        [-0.6249865013939683, -0.37500988759547166, 0.25],
        [-0.6249745852346407, -0.12500952625699208, 0.25],
        [-0.8749996666010157, -1.124999996317431, 0.25],
        [-0.8750022743013857, -1.3750002197360136, 0.25],
        [-0.8750026228707772, -1.625000293946456, 0.25],
        [-0.875000683856389, -1.8750005431133077, 0.25],
        [-0.8750003374682109, -2.125000809975315, 0.25],
        [-1.124990544183853, 2.3500004688132425, 0.25],
        [-0.6250000300796363, -2.32500130625383, 0.25],
        [-1.1249953168168734, 2.0999996600800737, 0.25],
        [-0.6249995885987827, -2.074999923989746, 0.25]
    ]

    obstacle_list_2 = [[-1.3749988998291784, 1.375000128910268, 0.25, 0.25, 1.9735661080454134e-08],
                       [-1.3750000884965963, 1.625000432029795, 0.25, 0.25, 8.211307707899254e-08],
                       [-1.375000230425759, 1.875000805427043, 0.25, 0.25, -2.4020823895257e-07],
                       [-1.3750039862386922, 2.125002489301276, 0.25, 0.25, -5.904768261757853e-08],
                       [-1.375007027990223, 1.1249963851911506, 0.25, 0.25, 1.1339679431489447e-05],
                       [-1.1250034221984053, 1.1250076953832377, 0.25, 0.25, 4.647045731910574e-05],
                       [-0.8749892472055414, 1.125015403398453, 0.25, 0.25, 5.903712248616574e-06],
                       [-0.8750000670440002, 0.8750106288420382, 0.25, 0.25, -1.4275306204685798e-05],
                       [-0.8750022838851056, 0.6250103154977967, 0.25, 0.25, -1.4463977517448188e-05],
                       [-0.8749885064872707, 0.3750001428132739, 0.25, 0.25, -9.411429039573819e-05],
                       [-0.8749978362024249, 0.12499923812843783, 0.25, 0.25, -9.486154460594638e-05],
                       [-0.8749987578926902, -0.12501242744493973, 0.25, 0.25, 4.813394382044834e-08],
                       [-1.1250026391592498, -0.12500134394135037, 0.25, 0.25, -1.5486476069577747e-06],
                       [-1.3750037733287015, -0.12499925336079447, 0.25, 0.25, -1.2155567678527916e-07],
                       [-0.8749998576763511, -0.8749992918358481, 0.25, 0.25, 3.241037769204721e-06],
                       [-1.1250008202302646, -0.8749984416690622, 0.25, 0.25, 1.4828153322353408e-06],
                       [-1.3750013672761283, -0.8750003131859605, 0.25, 0.25, -2.696376686353382e-07],
                       [-0.624991282832163, -0.8750109533594129, 0.25, 0.25, -9.108294642649644e-07],
                       [-0.6249961585746708, -0.6250102520458617, 0.25, 0.25, -4.444369647584517e-07],
                       [-0.6249865013939683, -0.37500988759547166, 0.25, 0.25, -9.249245979583684e-07],
                       [-0.6249745852346407, -0.12500952625699208, 0.25, 0.25, 1.1112701874685841e-06],
                       [-0.8749996666010157, -1.124999996317431, 0.25, 0.25, -5.656139334819719e-07],
                       [-0.8750022743013857, -1.3750002197360136, 0.25, 0.25, -7.570795001227611e-08],
                       [-0.8750026228707772, -1.625000293946456, 0.25, 0.25, -6.934637988706109e-08],
                       [-0.875000683856389, -1.8750005431133077, 0.25, 0.25, 1.194317538417912e-06],
                       [-0.8750003374682109, -2.125000809975315, 0.25, 0.25, 7.52700819988477e-08],
                       [-1.124990544183853, 2.3500004688132425, 0.25, 0.25, 1.6852396775407579e-07],
                       [-0.6250000300796363, -2.32500130625383, 0.25, 0.25, -6.128405607827895e-08],
                       [-1.1249953168168734, 2.0999996600800737, 0.25, 0.25, -2.0100946659164694e-08],
                       [-0.6249995885987827, -2.074999923989746, 0.25, 0.25, 1.0713491147601574e-07]]
                       # [-2.35, -1.0, 0.1, 0.1, 0.0], [-2.35, -0.9, 0.1, 0.1, 0.0], [-2.35, -0.8, 0.1, 0.1, 0.0],
                       # [-2.35, -0.7, 0.1, 0.1, 0.0], [-2.35, -0.6, 0.1, 0.1, 0.0], [-2.35, -0.5, 0.1, 0.1, 0.0],
                       # [-2.35, -0.4, 0.1, 0.1, 0.0], [-2.35, -0.3, 0.1, 0.1, 0.0], [-2.35, -0.2, 0.1, 0.1, 0.0],
                       # [-2.35, -0.1, 0.1, 0.1, 0.0], [-2.35, 0.0, 0.1, 0.1, 0.0], [-2.35, 0.1, 0.1, 0.1, 0.0],
                       # [-2.35, 0.2, 0.1, 0.1, 0.0], [-2.35, 0.3, 0.1, 0.1, 0.0], [-2.35, 0.4, 0.1, 0.1, 0.0],
                       # [-2.35, 0.5, 0.1, 0.1, 0.0], [-2.35, 0.6, 0.1, 0.1, 0.0], [-2.25, -1.0, 0.1, 0.1, 0.0],
                       # [-2.25, 0.6, 0.1, 0.1, 0.0], [-2.15, -1.0, 0.1, 0.1, 0.0], [-2.15, 0.6, 0.1, 0.1, 0.0],
                       # [-2.05, -1.0, 0.1, 0.1, 0.0], [-2.05, 0.6, 0.1, 0.1, 0.0], [-1.95, -1.0, 0.1, 0.1, 0.0],
                       # [-1.95, 0.6, 0.1, 0.1, 0.0], [-1.85, -1.0, 0.1, 0.1, 0.0], [-1.85, 0.6, 0.1, 0.1, 0.0],
                       # [-1.75, -1.0, 0.1, 0.1, 0.0], [-1.75, 0.6, 0.1, 0.1, 0.0], [-1.65, -1.0, 0.1, 0.1, 0.0],
                       # [-1.65, 0.6, 0.1, 0.1, 0.0], [-1.55, -1.0, 0.1, 0.1, 0.0], [-1.55, 0.6, 0.1, 0.1, 0.0],
                       # [-1.45, -1.0, 0.1, 0.1, 0.0], [-1.45, 0.6, 0.1, 0.1, 0.0], [-1.35, -1.0, 0.1, 0.1, 0.0],
                       # [-1.35, 0.6, 0.1, 0.1, 0.0], [-1.25, -1.0, 0.1, 0.1, 0.0], [-1.25, 0.6, 0.1, 0.1, 0.0],
                       # [-1.15, -1.0, 0.1, 0.1, 0.0], [-1.15, 0.6, 0.1, 0.1, 0.0], [-1.05, -1.0, 0.1, 0.1, 0.0],
                       # [-1.05, 0.6, 0.1, 0.1, 0.0], [-0.95, -1.0, 0.1, 0.1, 0.0], [-0.95, 0.6, 0.1, 0.1, 0.0],
                       # [-0.85, -1.0, 0.1, 0.1, 0.0], [-0.85, 0.6, 0.1, 0.1, 0.0], [-0.75, -1.0, 0.1, 0.1, 0.0],
                       # [-0.75, 0.6, 0.1, 0.1, 0.0], [-0.65, -1.0, 0.1, 0.1, 0.0], [-0.65, 0.6, 0.1, 0.1, 0.0],
                       # [-0.55, -1.0, 0.1, 0.1, 0.0], [-0.55, -0.9, 0.1, 0.1, 0.0], [-0.55, -0.8, 0.1, 0.1, 0.0],
                       # [-0.55, -0.7, 0.1, 0.1, 0.0], [-0.55, -0.6, 0.1, 0.1, 0.0], [-0.55, -0.5, 0.1, 0.1, 0.0],
                       # [-0.55, -0.4, 0.1, 0.1, 0.0], [-0.55, -0.3, 0.1, 0.1, 0.0], [-0.55, -0.2, 0.1, 0.1, 0.0],
                       # [-0.55, -0.1, 0.1, 0.1, 0.0], [-0.55, 0.0, 0.1, 0.1, 0.0], [-0.55, 0.1, 0.1, 0.1, 0.0],
                       # [-0.55, 0.2, 0.1, 0.1, 0.0], [-0.55, 0.3, 0.1, 0.1, 0.0], [-0.55, 0.4, 0.1, 0.1, 0.0],
                       # [-0.55, 0.5, 0.1, 0.1, 0.0], [-0.55, 0.6, 0.1, 0.1, 0.0]]

    data_array = []

    dStar_runtime, dStar_length = 0.0, 0.0
    dijkstra_runtime, dijkstra_length = 0.0, 0.0
    aStarPlanner_runtime, aStarPlanner_length = 0.0, 0.0
    rRTStarReedsShepp_runtime, rRTStarReedsShepp_length = 0.0, 0.0

    for i in range(0, 30, 1):
        runtime_1, length_1 = runtimeAndPathLengthCalculate(path_planning_DStar, start, goal, obstacle_list_1,
                                                            rand_area_x_m, rand_area_y_m)
        dStar_runtime += runtime_1
        dStar_length += float(length_1)

        runtime_2, length_2 = runtimeAndPathLengthCalculate(path_planning_Dijkstra, start, goal, obstacle_list_1,
                                                            rand_area_x_m, rand_area_y_m)
        dijkstra_runtime += runtime_2
        dijkstra_length += float(length_2)

        runtime_3, length_3 = runtimeAndPathLengthCalculate(path_planning_AStarPlanner, start, goal, obstacle_list_1,
                                                            rand_area_x_m, rand_area_y_m)
        aStarPlanner_runtime += runtime_3
        aStarPlanner_length += float(length_3)

        runtime_4, length_4 = runtimeAndPathLengthCalculate(path_planning_RRTStarReedsShepp, start, goal,
                                                            obstacle_list_2, 0, 0)
        rRTStarReedsShepp_runtime += runtime_4
        rRTStarReedsShepp_length += float(length_4)

    data_array.append(('DStar', dStar_runtime / 30, dStar_length / 30))
    data_array.append(('Dijkstra', dijkstra_runtime / 30, dijkstra_length / 30))
    data_array.append(('RRTStarReedsShepp', rRTStarReedsShepp_runtime / 30, rRTStarReedsShepp_length / 30))
    data_array.append(('AStarPlanner', aStarPlanner_runtime / 30, aStarPlanner_length / 30))

    drawCompareChart(data_array)
