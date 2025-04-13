import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

# === Load and parse the poses.txt ===

def extract_vector_from_line(line):
    # Extract text inside square brackets
    match = re.search(r'\[([^\]]+)\]', line)
    if match:
        numbers = match.group(1)
        return np.array([float(n) for n in numbers.split(',')])
    else:
        raise ValueError(f"Could not parse line: {line}")

def parse_poses(filename):
    poses = []
    with open(filename, 'r') as f:
        lines = f.readlines()

    for i in range(0, len(lines), 2):
        rvec = extract_vector_from_line(lines[i])
        tvec = extract_vector_from_line(lines[i + 1])

        R, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec

        poses.append(T)

    return poses


# === Accumulate camera positions ===

def get_camera_positions(poses):
    positions = []
    current_pose = np.eye(4)

    for pose in poses:
        current_pose = current_pose @ np.linalg.inv(pose)  # because pose is camera-to-world, invert to accumulate motion
        position = current_pose[:3, 3]
        positions.append(position)

    return np.array(positions)

# === Visualize trajectory ===

def plot_trajectory(positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Camera trajectory')
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='r', s=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Camera Trajectory')
    ax.legend()
    ax.view_init(elev=30, azim=120)

    plt.show()

# === Run everything ===

if __name__ == '__main__':
    poses = parse_poses("poses.txt")
    positions = get_camera_positions(poses)
    plot_trajectory(positions)
