"""
python plot_data.py

Author: @Zi-ang-Cao
"""

import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

def read_data(file_path='./data_files/que_1.txt', question_idx="1_a"):
    # Open and read the lines of the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Initialize lists to store the data
    X_values = []

    if "1" in question_idx:
        Y_dim = 3 * 2  # 3 for task space actual trajectory, 3 for task space desired trajectory
    elif "2" in question_idx:
        Y_dim = 3 * 2 + 2   # task space (xyz) + joint4, joint6
    else:
        Y_dim = 0

    Y_values_list = [ [] for i in range(Y_dim)]
    subTitle_values = [ " " for i in range(Y_dim)]

    title_values = " "
    if question_idx in ["1_a", "1_c"]:
        title_values = f"Operational Space PD Controllers \n[Blue] Actual Trajectory vs [Red] Desired Trajectory"
    elif "2" in question_idx:
        title_values = "Null Space Control"
    else:
        raise ValueError("Invalid question_idx")
    
    i = 0  # Start after the header
    group_size = 1
    while i <= (len(lines) - group_size):
        line_content = lines[i].strip().split()
        # Extract X_values
        x = float(line_content[0])
        X_values.append(x)
        # Extract Y_values
        for idx in range(Y_dim):
            if idx < 6:
                # In task space -- actual trajectory (0-2) + expected trajectory (3-5)
                Y_values_list[idx].append(float(line_content[idx+1]))
            else:
                if "2" in question_idx:
                    # Find joint 4 and joint 6
                    if idx == 6:
                        Y_values_list[idx].append(float(line_content[-4]) * 180 / np.pi)  # joint 4 in [degree]
                    elif idx == 7:
                        Y_values_list[idx].append(float(line_content[-2]) * 180 / np.pi)  # joint 6 in [degree]
                    else:
                        continue
                else:
                    continue


        i += group_size

    return X_values, Y_values_list, subTitle_values, title_values
    

# Function to plot the data
def plot_data(q_values, Y_values_list, subTitle_values, title_values, output_file_path='que_1.png', question_idx="2"):
    X_label = 'control time'
    X_unit = 's'

    if "1" in question_idx:
        # Y_dim = len(Y_values_list) / 2
        Y_dim = 3
        axis = ['x', 'y', 'z']
    else:
        Y_dim = 5
        axis = ['x', 'y', 'z', 'Joint 4', 'Joint 6']
        joint_limits_high = [-30, 210]
        joint_limits_low = [-170, 0]


    fig, axs = plt.subplots(Y_dim, 1, figsize=(10 , 8 * Y_dim /3))  # Create a figure with 3 subplots
    for i in range(Y_dim):
        # Plot actual trajectory in blue
        # Plot desired trajectory in red
        Y_unit = 'm' if i < 3 else 'radians'

        if i < 3: 
            axs[i].plot(q_values, Y_values_list[i], marker='o', linestyle='-', color='blue', label=f'Real Traj {axis[i]}')
            axs[i].plot(q_values, Y_values_list[i + 3], marker='*', linestyle='-', color='red', label=f'Desired Traj {axis[i]}')
            Y_label = f'EEF trajectory in {axis[i]}'
            axs[i].set_title(f'{Y_label} vs {X_label}')
            axs[i].set_xlabel(f'{X_label} ({X_unit})')
            axs[i].set_ylabel(f'{Y_label} ({Y_unit})')

            

            # plt.legend(loc='upper right')  # You can specify the location of the legend

        elif 3 <= i and i < 5:
            axs[i].plot(q_values, Y_values_list[i + 3], marker='o', linestyle='-', color='blue', label=f'joint Traj')

            boundary_high = [joint_limits_high[i-3] for q in q_values]
            boundary_low = [joint_limits_low[i-3] for q in q_values]
            axs[i].plot(q_values, boundary_high, marker='o', linestyle='-', color='grey', label=f'joint limits')
            axs[i].plot(q_values, boundary_low, marker='o', linestyle='-', color='grey')
            
            Y_label = f'{axis[i]}'
            axs[i].set_title(f'{Y_label} vs {X_label}')
            axs[i].set_xlabel(f'{X_label} ({X_unit})')
            axs[i].set_ylabel(f'{Y_label} ({Y_unit})')
        else:
            continue

        axs[i].legend(loc='upper right')


    # plt.suptitle(f'Simulated {mode} vs {X_label} ({intersted_range})')  # Set figure title
    plt.suptitle(f'{title_values}')  # Set figure title
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust subplots to fit the title

    parent_folder = os.path.dirname(output_file_path)
    os.makedirs(parent_folder, exist_ok=True)
    plt.savefig(output_file_path)  # Save the plot as an image



# Change directory to the desired path
os.chdir('/Users/zi-angcao/03_ResearchRepo/SAI2/OpenSai/STF_CS225A_HW/homework/hw3')

# for question_idx in ["1_a", "1_c"]:
# for question_idx in ["2"]:
for question_idx in ["2_d", "2_e", "2_f", "2_g"]:
    file_path=f'./data_files/que_{question_idx}.txt'
    output_file_path=f'./img_out/que_{question_idx}.png'
    X_values, Y_values_list, subTitle_values, title_values = read_data(file_path, question_idx)
    plot_data(X_values, Y_values_list, subTitle_values, title_values, output_file_path, question_idx)