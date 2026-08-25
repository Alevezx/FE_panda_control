import os
import time
import xml.etree.ElementTree as ET

# Drake imports
from pydrake.all import StartMeshcat, DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.geometry import MeshcatVisualizer, Sphere, Rgba
from pydrake.math import RigidTransform

# ==============================================================================
# 1. CUSTOM TRAJECTORY PARSERS
# ==============================================================================
def parse_trajectory_xml(file_path, stride=10):
    """
    Parses the custom XML file containing keypoints and local time stamps.
    Converts resetting segment times into a continuous absolute timeline.
    Downsamples by 'stride' (takes 1 sample out of every N samples).
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except ET.ParseError:
        with open(file_path, 'r') as f:
            xml_data = f"<root>\n" + f.read() + "\n</root>"
        root = ET.fromstring(xml_data)

    trajectory = []
    absolute_time = 0.0
    prev_local_time = 0.0

    all_keypoints = root.findall('.//keypoint')

    for i, keypoint in enumerate(all_keypoints):
        t_local = float(keypoint.attrib['time'])

        # Detect if timer reset (new movement segment)
        if t_local >= prev_local_time:
            dt = t_local - prev_local_time
        else:
            dt = t_local  # Timer restarted from 0.0

        absolute_time += dt
        prev_local_time = t_local

        # Downsample according to stride (always keep last frame)
        if i % stride == 0 or i == len(all_keypoints) - 1:
            points = keypoint.findall('point')
            sorted_points = sorted(points, key=lambda p: int(p.attrib['id']))
            q_values = [float(p.text) for p in sorted_points]
            
            trajectory.append({'time': absolute_time, 'q': q_values})

    return trajectory


def parse_skeleton_xml(file_path):
    """
    Parses the skeleton XML containing spatial points over time.
    Replaces 'nan' values with None for easy filtering.
    """
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except ET.ParseError:
        with open(file_path, 'r') as f:
            xml_data = f"<root>\n" + f.read() + "\n</root>"
        root = ET.fromstring(xml_data)

    # Find keypoints whether root is <skeleton> or a wrapper <root>
    keypoints = root.findall('.//keypoint')
    skeleton_trajectory = []

    for keypoint in keypoints:
        time_stamp = float(keypoint.attrib['time']) #[cite: 2]
        points_dict = {}
        
        for point in keypoint.findall('point'):
            pid = int(point.attrib['id']) #[cite: 2]
            text_vals = point.text.strip().split() #[cite: 2]
            
            if 'nan' in text_vals[0].lower(): #[cite: 2]
                points_dict[pid] = None
            else:
                points_dict[pid] = [float(v) for v in text_vals]
                
        skeleton_trajectory.append({'time': time_stamp, 'points': points_dict})
        
    # Sort chronologically just in case
    skeleton_trajectory.sort(key=lambda x: x['time'])
    return skeleton_trajectory


# ==============================================================================
# 2. MAIN EXECUTION PIPELINE
# ==============================================================================
def main():
    # Base workspace path
    base_dir = "/home/alessandro/Desktop/tesi/codici_cpp/collision_avoidance_virtual"
    urdf_path = os.path.join(base_dir, "urdf/panda_visual.urdf")
    xml_path = os.path.join(base_dir, "logs/q_robot.xml")
    skeleton_path = os.path.join(base_dir, "skeleton/skeleton_coords_fixed.xml")

    # Start the MeshCat server backend
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    # Initialize physics plant and geometry layout engine
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    # Initialize model parser
    parser = Parser(plant)
    parser.package_map().Add("franka_description", base_dir)
    
    # Load model and capture its unique instance index
    model_instances = parser.AddModels(urdf_path)
    panda_model = model_instances[0]

    # WELD BASE LINK
    base_frame = plant.GetFrameByName("panda_link0", panda_model)
    plant.WeldFrames(plant.world_frame(), base_frame)

    plant.Finalize()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)

    # Parse trajectories
    print(f"Reading tracking targets from: {xml_path}")
    traj_data = parse_trajectory_xml(xml_path, stride=10)
    
    print(f"Reading skeleton tracking from: {skeleton_path}")
    try:
        skeleton_data = parse_skeleton_xml(skeleton_path)
        print(f"✔ Parsed {len(skeleton_data)} skeleton keypoints successfully.")
    except Exception as e:
        print(f"⚠ Failed to load skeleton tracking: {e}")
        skeleton_data = []

    # Initialize Skeleton Spheres in Meshcat
    if skeleton_data:
        # Determine the maximum ID to instantiate enough geometric spheres
        max_id = max([max(frame['points'].keys()) for frame in skeleton_data])
        for i in range(max_id + 1):
            # Create a green sphere of radius 0.1 for every potential point ID
            meshcat.SetObject(f"skeleton/point_{i}", Sphere(0.1), Rgba(0.2, 0.8, 0.2, 1.0))

    # Print connection URL for display access
    print(f"\nServer running! Open this URL in your web browser: {meshcat.web_url()}\n")
    print("Waiting 3 seconds for you to open the browser tab before starting animation...")
    time.sleep(3)

    # Playback sequence execution loop
    print("Playing trajectory simulation...")
    prev_time = None
    current_skel_idx = 0

    for step in traj_data:
        curr_time = step['time']

        # Dynamically sleep to match real-time
        if prev_time is not None:
            dt = curr_time - prev_time
            if dt > 0:
                time.sleep(dt)

        prev_time = curr_time

        # ---------------------------------------------------------
        # UPDATE SKELETON
        # ---------------------------------------------------------
        if skeleton_data:
            # Advance skeleton index if the simulation time passes the next keypoint
            while current_skel_idx < len(skeleton_data) - 1 and skeleton_data[current_skel_idx + 1]['time'] <= curr_time:
                current_skel_idx += 1
            
            # Apply current frame poses
            current_frame = skeleton_data[current_skel_idx]['points']
            for pid, coords in current_frame.items():
                if coords is None:
                    # Hide 'nan' points below the rendering ground
                    meshcat.SetTransform(f"skeleton/point_{pid}", RigidTransform([0, 0, -1000]))
                else:
                    # Move valid point to its spatial coordinates
                    meshcat.SetTransform(f"skeleton/point_{pid}", RigidTransform(coords))

        # ---------------------------------------------------------
        # UPDATE ROBOT
        # ---------------------------------------------------------
        q_arm = step['q']
        plant.SetPositions(plant_context, q_arm)
        
        # Force interface redraw
        visualizer.ForcedPublish(visualizer.GetMyContextFromRoot(diagram_context))

    print("Trajectory playback finished.")

    # Keep script context open to allow browser interactions
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nVisualizer stopped.")

if __name__ == "__main__":
    main()