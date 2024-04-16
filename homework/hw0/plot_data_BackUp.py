# Let's process the data from the file uploaded to extract q3, m11, m22, and m33.

import pandas as pd



# def read_data(file_path='./data_files/q2-e-i.txt', q_index=2):


# File path to the uploaded text data
file_path = './data_files/q2-e-i.txt'

# Open and read the lines of the file
with open(file_path, 'r') as file:
    lines = file.readlines()

# Initialize lists to store the data
q3_values = []
m11_values = []
m22_values = []
m33_values = []

# Process each group of four lines after the header
Title_values = lines[0].strip().split()
intersted_range = Title_values[2:5]
intersted_range = " ".join(intersted_range)
i = 1  # Start after the header
while i < len(lines) - 3:
    q_values = lines[i].strip().split()
    m_matrix1 = lines[i + 1].strip().split()
    m_matrix2 = lines[i + 2].strip().split()
    m_matrix3 = lines[i + 3].strip().split()
    
    # Extract q3 and the diagonal of the mass matrix (m11, m22, m33)
    q3 = float(q_values[2])
    m11 = float(m_matrix1[0])
    m22 = float(m_matrix2[1])
    m33 = float(m_matrix3[2])
    
    # Append to the lists
    q3_values.append(q3)
    m11_values.append(m11)
    m22_values.append(m22)
    m33_values.append(m33)
    
    # Move to the next group of four lines
    i += 4

# Now we'll create the plots
import matplotlib.pyplot as plt

fig, axs = plt.subplots(3, 1, figsize=(10, 8))

# Plotting m11 vs q3
axs[0].plot(q3_values, m11_values, marker='o', linestyle='-')
axs[0].set_title('m11 vs thta_3')
axs[0].set_xlabel('thta_3 (radians)')
axs[0].set_ylabel('m11')

# Plotting m22 vs q3
axs[1].plot(q3_values, m22_values, marker='o', linestyle='-')
axs[1].set_title('m22 vs thta_3')
axs[1].set_xlabel('thta_3 (radians)')
axs[1].set_ylabel('m22')

# Plotting m33 vs q3
axs[2].plot(q3_values, m33_values, marker='o', linestyle='-')
axs[2].set_title('m33 vs thta_3')
axs[2].set_xlabel('thta_3 (radians)')
axs[2].set_ylabel('m33')

plt.suptitle(f'Simulated M vs thta_3 ({intersted_range})')  # Set figure title
plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust subplots to fit the title
plt.savefig('q2-e-i.png')  # Save the plot as an image
# plt.show()
