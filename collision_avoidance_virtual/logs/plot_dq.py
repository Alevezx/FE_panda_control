import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import numpy as np

filename = 'dq_robot.xml'

tree = ET.parse(filename)
root = tree.getroot()

keypoints = root.findall('.//keypoint')
N = len(keypoints)

time = np.zeros(N)
q = np.zeros((N, 7))  # One column for each point id

for i, kp in enumerate(keypoints):
    # Time
    time[i] = float(kp.attrib['time'])

    # Seven point values
    pts = kp.findall('point')
    for j in range(7):
        q[i, j] = float(pts[j].text)

# Plot all seven trajectories
plt.figure()
t = np.linspace(0, (N - 1) / 1000, N)
plt.plot(t, q, linewidth=1.5)

plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Joint speed')
plt.title('Robot joint speed')

# Commented legend
# plt.legend([f'Joint {i}' for i in range(7)], loc='best')

# Vertical lines (translated from MATLAB xline)
# vline_times = [3, 4, 7, 9, 11]
# vline_color = (0.4, 0.4, 0.4)   # Dark grey
# vline_style = '--'
# vline_width = 1.2
#
# for vt in vline_times:
#     plt.axvline(x=vt, color=vline_color, linestyle=vline_style, linewidth=vline_width, label=f't = {vt}')

plt.show()