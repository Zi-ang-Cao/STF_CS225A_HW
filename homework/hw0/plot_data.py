"""
python plot_data.py

Author: @Zi-ang-Cao
"""

import pandas as pd
import matplotlib.pyplot as plt
import os

def read_data(file_path='./data_files/q2-e-i.txt', joint_id=2, mode="M"):
    # Open and read the lines of the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Initialize lists to store the data
    q_values = []

    if mode == "M":
        M11_values = []
        M22_values = []
        M33_values = []
    else:
        G1_values = []
        G2_values = []
        G3_values = []

    # Process each group of four lines after the header
    Title_values = lines[0].strip().split()
    if joint_id == 3:
        intersted_range = Title_values[2:5]
    elif joint_id == 2:
        intersted_range = Title_values[1:4]
    else:
        raise ValueError("Invalid joint_id")

    intersted_range = " ".join(intersted_range)
    i = 1  # Start after the header

    group_size = 4 if mode == "M" else 2
    while i <= (len(lines) - group_size):
        robot_q = lines[i].strip().split()
        # Extract q_variable
        q_values.append(float(robot_q[joint_id-1]))

        if mode == "M":
            m_matrix1 = lines[i + 1].strip().split()
            m_matrix2 = lines[i + 2].strip().split()
            m_matrix3 = lines[i + 3].strip().split()
            # Extract q_variable and the diagonal of the mass matrix (M11, M22, M33)
            M11 = float(m_matrix1[0])
            M22 = float(m_matrix2[1])
            M33 = float(m_matrix3[2])
            M11_values.append(M11)
            M22_values.append(M22)
            M33_values.append(M33)
            # Move to the next group of four lines
        elif mode == "G":
            G_matrix = lines[i + 1].strip().split()
            G1 = float(G_matrix[0])
            G2 = float(G_matrix[1])
            G3 = float(G_matrix[2])
            G1_values.append(G1)
            G2_values.append(G2)
            G3_values.append(G3)

        i += group_size

    if mode == "M":
        return q_values, M11_values, M22_values, M33_values, intersted_range
    elif mode == "G":
        return q_values, G1_values, G2_values, G3_values, intersted_range


# Function to plot the data
def plot_data(q_values, Y1_values, Y2_values, Y3_values, intersted_range, joint_id=2, mode="M", output_file_path='q2-e-i.png'):
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    if joint_id == 3:
        X_label = 'thta_3'    
        X_unit = 'radians'
    elif joint_id == 2:
        X_label = 'd_2'    
        X_unit = 'm'
    else:
        raise ValueError("Invalid joint_id")
    
    for i, Y_values in enumerate([Y1_values, Y2_values, Y3_values]):
        # Plotting M11 vs q_variable
        axs[i].plot(q_values, Y_values, marker='o', linestyle='-', color='purple')
        if mode == "M":
            Y_label = f'M{i+1}{i+1}'
        elif mode == "G":
            Y_label = f'G{i+1}'
        
        axs[i].set_title(f'{Y_label} vs {X_label}')
        axs[i].set_xlabel(f'{X_label} ({X_unit})')
        axs[i].set_ylabel(f'{Y_label}')

    plt.suptitle(f'Simulated {mode} vs {X_label} ({intersted_range})')  # Set figure title
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust subplots to fit the title

    parent_folder = os.path.dirname(output_file_path)
    os.makedirs(parent_folder, exist_ok=True)
    plt.savefig(output_file_path)  # Save the plot as an image

file_path = './data_files/q2-e-i.txt'
joint_id = 3
mode = "M"
output_file_path = './img_out/q2-e-i.png'

q_values, Y1_values, Y2_values, Y3_values, intersted_range = read_data(file_path, joint_id, mode)
plot_data(q_values, Y1_values, Y2_values, Y3_values, intersted_range, joint_id, mode, output_file_path)


file_path = './data_files/q2-e-ii.txt'
joint_id = 2
mode = "M"
output_file_path = './img_out/q2-e-ii.png'

q_values, Y1_values, Y2_values, Y3_values, intersted_range = read_data(file_path, joint_id, mode)
plot_data(q_values, Y1_values, Y2_values, Y3_values, intersted_range, joint_id, mode, output_file_path)



file_path = './data_files/q2-f-i.txt'
joint_id = 3
mode = "G"
output_file_path = './img_out/q2-f-i.png'

q_values, Y1_values, Y2_values, Y3_values, intersted_range = read_data(file_path, joint_id, mode)
plot_data(q_values, Y1_values, Y2_values, Y3_values, intersted_range, joint_id, mode, output_file_path)


file_path = './data_files/q2-f-ii.txt'
joint_id = 2
mode = "G"
output_file_path = './img_out/q2-f-ii.png'

q_values, Y1_values, Y2_values, Y3_values, intersted_range = read_data(file_path, joint_id, mode)
plot_data(q_values, Y1_values, Y2_values, Y3_values, intersted_range, joint_id, mode, output_file_path)