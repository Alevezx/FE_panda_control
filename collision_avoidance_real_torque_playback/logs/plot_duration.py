import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt

filename = 'dist_segment.xml'

# Parse XML structure
tree = ET.parse(filename)
root = tree.getroot()

keypoints = root.findall('.//keypoint')

# Read time attributes and stop_duration element values
time = [float(kp.attrib['time']) for kp in keypoints]
stop_duration = [float(kp.find('stop_duration').text) for kp in keypoints]

# Plot data points with markers and connecting line
plt.figure()
plt.plot(time, stop_duration, 'o-')  # Combines marker 'o' and line plot

plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Stop duration (s)')

# Optional axis limits (from MATLAB commented line)
# plt.xlim(0, 12)
# plt.ylim(0, 0.5)

plt.show()