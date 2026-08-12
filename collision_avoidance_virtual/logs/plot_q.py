import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np

filename = 'q_robot.xml'

# Parse XML document
tree = ET.parse(filename)
root = tree.getroot()

# Find all keypoint elements
keypoints = root.findall('.//keypoint')
N = len(keypoints)

time = np.zeros(N)
q = np.zeros((N, 7))  # One column for each point id

for i, kp in enumerate(keypoints):
    # Extract time attribute
    time[i] = float(kp.attrib['time'])

    # Extract seven point values
    pts = kp.findall('point')
    for j in range(7):
        q[i, j] = float(pts[j].text)

# Plot all seven trajectories
plt.figure()
t = np.linspace(0, (N - 1) / 1000, N)  # Equivalent to 0:0.001:(N-1)/1000

plt.plot(t, q, linewidth=1.5)

plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Joint position')
plt.title('Robot joint positions')

plt.legend([f'Joint {i}' for i in range(7)], loc='best')
plt.show()