#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import sys
import os

def get_transform_matrix(position, orientation):
    """Converts position dict and orientation dict to 4x4 matrix."""
    t = np.eye(4)
    t[:3, 3] = [position['x'], position['y'], position['z']]
    r = R.from_quat([orientation['x'], orientation['y'], orientation['z'], orientation['w']])
    t[:3, :3] = r.as_matrix()
    return t

def main():
    # 1. Determine Files
    scenario_file = 's_curve_scenario.yaml'
    dataset_file = 's_curve_noisy_dataset.yaml'
    
    if len(sys.argv) > 1:
        dataset_file = sys.argv[1]
    
    print(f"Loading Scenario: {scenario_file}")
    print(f"Loading Dataset: {dataset_file}")

    if not os.path.exists(dataset_file):
        print(f"Error: Dataset file '{dataset_file}' not found.")
        return

    # 2. Load Data
    with open(scenario_file, 'r') as f:
        scenario_data = yaml.safe_load(f)
        
    with open(dataset_file, 'r') as f:
        dataset_data = yaml.safe_load(f)

    # 3. Prepare Plot
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # --- Plot A: Ground Truth Tags (Fixed World) ---
    gt_tags_x = []
    gt_tags_y = []
    gt_tags_labels = []
    
    for tag_name, data in scenario_data['tags'].items():
        gt_tags_x.append(data['position']['x'])
        gt_tags_y.append(data['position']['y'])
        gt_tags_labels.append(tag_name)
        
    ax.scatter(gt_tags_x, gt_tags_y, c='red', s=150, marker='X', label='Tag Real (World)')
    
    # Annotate Tag Names
    for i, txt in enumerate(gt_tags_labels):
        ax.annotate(txt, (gt_tags_x[i], gt_tags_y[i]), xytext=(5, 5), textcoords='offset points')

    # --- Plot B: Robot Path (Ground Truth from Dataset) ---
    robot_x = []
    robot_y = []
    
    # --- Plot C: Detected Points (Projected to World) ---
    detected_x = []
    detected_y = []
    




    print(f"Processing {len(dataset_data['frames'])} frames...")

    for frame in dataset_data['frames']:
        # 1. Get Robot GT Pose (World -> Base)
        gt_pose = frame['ground_truth_pose']
        robot_x.append(gt_pose['position']['x'])
        robot_y.append(gt_pose['position']['y'])
        
        T_world_opt = get_transform_matrix(gt_pose['position'], gt_pose['orientation'])
        
        # 2. Process Detections
        detections = frame['detections']
        if not detections:
            continue
            
        for tag_id, det_pose in detections.items():
            # Pose of Tag relative to Optical Frame (T_optical_tag)
            T_opt_tag = get_transform_matrix(det_pose['position'], det_pose['orientation'])
            
            # Chain Transforms: World = T_world_base * T_base_opt * T_opt_tag
            # The 'position' in T_opt_tag is the vector from Camera to Tag.
            # We want that point in World Frame.
            
            T_world_tag_measured = T_world_opt @ T_opt_tag
            
            p_world = T_world_tag_measured[:3, 3]
            
            detected_x.append(p_world[0])
            detected_y.append(p_world[1])


    
    # Draw Camera Path
    ax.plot(robot_x, robot_y, 'g--', alpha=0.5, linewidth=1, label='Trajetória Câmera (GT)')
    
    # Draw Detections Cloud
    if detected_x:
        ax.scatter(detected_x, detected_y, c='blue', s=10, alpha=0.3, label='Detecções (Projetadas)')
    else:
        print("Warning: No detections found in dataset.")

    ax.set_title(f"Visualização do Dataset: {dataset_file}\n(Detecções projetadas no Mundo usando Pose Real do Robô)")
    ax.set_xlabel("World X (m)")
    ax.set_ylabel("World Y (m)")
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    output_filename = dataset_file.replace('.yaml', '_plot.png')
    plt.savefig(output_filename)
    print(f"Plot salvo em: {output_filename}")

if __name__ == "__main__":
    main()
