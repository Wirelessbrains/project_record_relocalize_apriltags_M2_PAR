import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from scipy.sparse import lil_matrix
import sys
import time

# ==========================================
# 1. SETUP AND DATA LOADING
# ==========================================
print("Loading measurement dataset...")
try:
    filename = 's_curve_dataset_large.yaml'
    with open(filename, 'r') as f:
        dataset = yaml.safe_load(f)
except Exception as e:
    print(f"Error opening file '{filename}': {e}")
    sys.exit(1)

ground_truth_traj = [] 
tag_id_map = {} 
next_tag_idx = 0
reverse_tag_map = {} 

obs_cam_idxs = []
obs_tag_idxs = []
obs_measurements = []

gt_rel_pos = [] 
gt_rel_rot = [] 

frames = dataset['frames']
num_frames = len(frames)

first_observation_data = None
first_frame_with_detection = -1

for i, frame in enumerate(frames):
    # Abs GT
    gt_p = frame['ground_truth_pose']['position']
    gt_q = frame['ground_truth_pose']['orientation']
    P_c_gt = np.array([gt_p['x'], gt_p['y'], gt_p['z']])
    R_c_gt = R.from_quat([gt_q['x'], gt_q['y'], gt_q['z'], gt_q['w']])
    ground_truth_traj.append(P_c_gt)

    # Extract Observations
    if frame.get('detections'):
        if first_frame_with_detection == -1:
             first_frame_with_detection = i
             
        for tag_name, pose_data in frame['detections'].items():
            if tag_name not in tag_id_map:
                tag_id_map[tag_name] = next_tag_idx
                reverse_tag_map[next_tag_idx] = tag_name
                next_tag_idx += 1
                
            meas_local = np.array([pose_data['position']['x'], pose_data['position']['y'], pose_data['position']['z']])

            if i == first_frame_with_detection and first_observation_data is None:
                first_observation_data = {
                    'cam_gt_rot': R_c_gt,
                    'meas_local': meas_local,
                    'tag_name': tag_name
                }
            
            obs_cam_idxs.append(i)
            obs_tag_idxs.append(tag_id_map[tag_name])
            obs_measurements.append(meas_local)
    
    # Prepare Relative Motion Data (Smoothing constraints)
    if i < num_frames - 1:
        curr = frames[i]['ground_truth_pose']
        nxt  = frames[i+1]['ground_truth_pose']
        
        R_c = R.from_quat([curr['orientation']['x'], curr['orientation']['y'], curr['orientation']['z'], curr['orientation']['w']])
        P_c = np.array([curr['position']['x'], curr['position']['y'], curr['position']['z']])
        R_n = R.from_quat([nxt['orientation']['x'], nxt['orientation']['y'], nxt['orientation']['z'], nxt['orientation']['w']])
        
        delta_P = R_c.inv().apply(np.array([nxt['position']['x'], nxt['position']['y'], nxt['position']['z']]) - P_c)
        delta_R = (R_c.inv() * R_n).as_rotvec()
        
        gt_rel_pos.append(delta_P)
        gt_rel_rot.append(delta_R)

# Convert to NumPy
obs_cam_idxs = np.array(obs_cam_idxs, dtype=np.int32)
obs_tag_idxs = np.array(obs_tag_idxs, dtype=np.int32)
obs_measurements = np.array(obs_measurements, dtype=np.float64)
ground_truth_traj = np.array(ground_truth_traj)
gt_rel_pos = np.array(gt_rel_pos)
gt_rel_rot = np.array(gt_rel_rot)

num_tags = len(tag_id_map)
num_observations = len(obs_cam_idxs)
offset_cam_rot = num_frames * 3

# ==========================================
# 3. WEIGHT CONFIGURATION
# ==========================================
WEIGHT_ANCHOR    = 1000.0  
WEIGHT_OBS       = 20.0    
WEIGHT_SMOOTH_P  = 80.0    
WEIGHT_SMOOTH_R  = 50.0    

# ==========================================
# 4. COST FUNCTIONS
# ==========================================

def fun_tags_only(x_tags, c_pos_fixed, c_rot_fixed):
    t_pos = x_tags.reshape(num_tags, 3)
    batch_c_pos = c_pos_fixed[obs_cam_idxs]
    batch_t_pos = t_pos[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot_fixed[obs_cam_idxs])
    P_diff = batch_t_pos - batch_c_pos
    P_local_est = batch_R_cam.inv().apply(P_diff)
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    return res_obs

def fun_traj_smooth(x_traj, t_pos_fixed):
    c_pos = x_traj[:offset_cam_rot].reshape(num_frames, 3)
    c_rot = x_traj[offset_cam_rot:].reshape(num_frames, 3)

    res_anchor_pos = (c_pos[first_frame_with_detection] - start_pos_vec) * WEIGHT_ANCHOR
    res_anchor_rot = (c_rot[first_frame_with_detection] - r_vec_start) * WEIGHT_ANCHOR
    
    batch_c_pos = c_pos[obs_cam_idxs]
    batch_t_pos = t_pos_fixed[obs_tag_idxs]
    batch_R_cam = R.from_rotvec(c_rot[obs_cam_idxs])
    
    P_diff = batch_t_pos - batch_c_pos
    P_local_est = batch_R_cam.inv().apply(P_diff)
    res_obs = (P_local_est - obs_measurements).ravel() * WEIGHT_OBS
    
    curr_pos = c_pos[:-1]; curr_rot = c_rot[:-1]
    next_pos = c_pos[1:];  next_rot = c_rot[1:]
    
    R_curr_obj = R.from_rotvec(curr_pos * 0 + curr_rot) 
    pred_next_pos = curr_pos + R_curr_obj.apply(gt_rel_pos)
    
    res_smooth_pos = (next_pos - pred_next_pos).ravel() * WEIGHT_SMOOTH_P
    res_smooth_rot = ((next_rot - curr_rot) - gt_rel_rot).ravel() * WEIGHT_SMOOTH_R

    return np.concatenate([res_anchor_pos, res_anchor_rot, res_obs, res_smooth_pos, res_smooth_rot])


# ==========================================
# 5. INITIALIZATION
# ==========================================
print("Generating initial estimate...")
if first_observation_data is None:
    print("ERROR: No tags detected."); sys.exit(1)

P_tag_anchor_world = np.zeros(3)
R_cam_anchor = first_observation_data['cam_gt_rot']
P_tag_local = first_observation_data['meas_local']
start_pos_vec = P_tag_anchor_world - R_cam_anchor.apply(P_tag_local)
r_vec_start = R_cam_anchor.as_rotvec()
idx_start = first_frame_with_detection

x0_cam_pos = np.zeros((num_frames, 3))
x0_cam_rot = np.zeros((num_frames, 3))
x0_tag_pos = np.zeros((num_tags, 3))

x0_cam_pos[idx_start] = start_pos_vec
x0_cam_rot[idx_start] = r_vec_start

for i in range(idx_start - 1, -1, -1):
    R_nxt_est = R.from_rotvec(x0_cam_rot[i+1])
    d_pos = gt_rel_pos[i]; d_rot = gt_rel_rot[i]
    R_inc = R.from_rotvec(d_rot)
    R_curr = R_nxt_est * R_inc.inv()
    x0_cam_pos[i] = x0_cam_pos[i+1] - R_curr.apply(d_pos)
    x0_cam_rot[i] = R_curr.as_rotvec()

for i in range(idx_start + 1, num_frames):
    R_prev_est = R.from_rotvec(x0_cam_rot[i-1])
    d_pos = gt_rel_pos[i-1]; d_rot = gt_rel_rot[i-1]
    R_inc = R.from_rotvec(d_rot)
    x0_cam_pos[i] = x0_cam_pos[i-1] + R_prev_est.apply(d_pos)
    x0_cam_rot[i] = (R_prev_est * R_inc).as_rotvec()

for i in range(num_observations):
    t_idx = obs_tag_idxs[i]; c_idx = obs_cam_idxs[i]
    if t_idx not in tag_id_map: continue
    R_cam_est = R.from_rotvec(x0_cam_rot[c_idx])
    tag_pos_world = x0_cam_pos[c_idx] + R_cam_est.apply(obs_measurements[i])
    t_name = reverse_tag_map.get(t_idx)
    if t_name == first_observation_data['tag_name']: x0_tag_pos[t_idx] = P_tag_anchor_world 
    else: x0_tag_pos[t_idx] = tag_pos_world
             
x0_raw_cam_pos = x0_cam_pos
x0_raw_cam_rot = x0_cam_rot

# ==========================================
# 6. SEQUENTIAL OPTIMIZATION
# ==========================================

# --- STEP 1: TAGS ---
print("\n[STEP 1/2] Optimizing Map (Tags)...")
x0_tags = x0_tag_pos.ravel()
sparsity_step1 = lil_matrix((num_observations * 3, num_tags * 3), dtype=int)
row_idx = 0
for i in range(num_observations):
    sparsity_step1[row_idx:row_idx+3, obs_tag_idxs[i]*3 : (obs_tag_idxs[i]+1)*3] = 1
    row_idx += 3

result_step1 = least_squares(
    fun_tags_only, x0_tags, jac_sparsity=sparsity_step1, verbose=0, method='trf', 
    x_scale='jac', args=(x0_raw_cam_pos, x0_raw_cam_rot)
)
est_tag_pos = result_step1.x.reshape(num_tags, 3)

# --- STEP 2: TRAJECTORY ---
print("\n[STEP 2/2] Optimizing Smooth Trajectory...")
x0_traj = np.hstack([x0_raw_cam_pos.ravel(), x0_raw_cam_rot.ravel()])
start_time_step2 = time.time()

n_smooth_res = (num_frames - 1) * 6 
m_step2 = 6 + num_observations * 3 + n_smooth_res 
n_step2 = num_frames * 6 

sparsity_step2 = lil_matrix((m_step2, n_step2), dtype=int)
row_idx = 0

sparsity_step2[0:6, idx_start*3 : (idx_start+1)*3] = 1
sparsity_step2[0:6, num_frames*3 + idx_start*3 : num_frames*3 + (idx_start+1)*3] = 1
row_idx += 6

for i in range(num_observations):
    c_idx = obs_cam_idxs[i]
    sparsity_step2[row_idx:row_idx+3, c_idx*3 : (c_idx+1)*3] = 1
    sparsity_step2[row_idx:row_idx+3, num_frames*3 + c_idx*3 : num_frames*3 + (c_idx+1)*3] = 1
    row_idx += 3

for i in range(num_frames - 1):
    idx_pos_i = i * 3; idx_pos_next = (i + 1) * 3
    idx_rot_i = num_frames * 3 + i * 3; idx_rot_next = num_frames * 3 + (i + 1) * 3
    
    sparsity_step2[row_idx:row_idx+3, idx_pos_i:idx_pos_i+3] = 1        
    sparsity_step2[row_idx:row_idx+3, idx_pos_next:idx_pos_next+3] = 1 
    sparsity_step2[row_idx:row_idx+3, idx_rot_i:idx_rot_i+3] = 1        
    row_idx += 3
    sparsity_step2[row_idx:row_idx+3, idx_rot_i:idx_rot_i+3] = 1        
    sparsity_step2[row_idx:row_idx+3, idx_rot_next:idx_rot_next+3] = 1 
    row_idx += 3

result_step2 = least_squares(
    fun_traj_smooth, x0_traj, jac_sparsity=sparsity_step2, verbose=0, method='trf', 
    ftol=1e-12, xtol=1e-12, x_scale='jac', args=(est_tag_pos,)
)
end_time_step2 = time.time()
print(f"STEP 2 DONE | Time: {end_time_step2 - start_time_step2:.2f}s | Cost: {result_step2.cost:.4f}")

est_cam_pos_opt = result_step2.x[:offset_cam_rot].reshape(num_frames, 3)
est_cam_rot_opt = result_step2.x[offset_cam_rot:].reshape(num_frames, 3)

# ==========================================
# 7. ALIGNMENT (PROCRUSTES)
# ==========================================
def align_procrustes(source, target):
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    H = source_centered.T @ target_centered
    U, S, Vt = np.linalg.svd(H)
    R_opt = Vt.T @ U.T
    if np.linalg.det(R_opt) < 0: Vt[2,:] *= -1; R_opt = Vt.T @ U.T
    t_opt = target_centroid - R_opt @ source_centroid
    return (R_opt @ source.T).T + t_opt, R_opt, t_opt

aligned_cam_pos, R_align, t_align = align_procrustes(est_cam_pos_opt, ground_truth_traj)
est_tag_pos_aligned = (R_align @ est_tag_pos.T).T + t_align

diff_aligned = aligned_cam_pos - ground_truth_traj
rmse_aligned = np.sqrt(np.mean(np.linalg.norm(diff_aligned, axis=1)**2))

print("\n" + "="*70)
print(f"{'FINAL RESULTS':^70}")
print("="*70)
print(f"Trajectory RMSE: {rmse_aligned:.4f} m")

# ==========================================
# 8. PROJECTED POSES (For Plotting)
# ==========================================
projected_poses = []
R_align_obj = R.from_matrix(R_align) 

for i in range(num_observations):
    c_idx = obs_cam_idxs[i]
    t_idx = obs_tag_idxs[i]
    r_vec_opt = est_cam_rot_opt[c_idx]
    R_opt_local = R.from_rotvec(r_vec_opt)
    R_cam_aligned = R_align_obj * R_opt_local 
    P_tag_world = est_tag_pos_aligned[t_idx]
    P_meas_local = obs_measurements[i]
    P_cam_proj = P_tag_world - R_cam_aligned.apply(P_meas_local)
    projected_poses.append(P_cam_proj)
projected_poses = np.array(projected_poses)


# ==========================================
# 9. PLOTTING
# ==========================================
plt.figure(figsize=(10, 8))

# Trajectories
plt.plot(ground_truth_traj[:, 0], ground_truth_traj[:, 1], 'k:', linewidth=1.5, alpha=0.6, label='Ground Truth Trajectory')
if len(projected_poses) > 0:
    plt.scatter(projected_poses[:, 0], projected_poses[:, 1], c='green', marker='^', s=30, alpha=0.5, label='Projected Poses (Measurements)')
plt.plot(aligned_cam_pos[:, 0], aligned_cam_pos[:, 1], 'b-', linewidth=2.5, label='Optimized Smooth Trajectory')

# Tags
plt.scatter(est_tag_pos_aligned[:, 0], est_tag_pos_aligned[:, 1], c='red', marker='x', s=100, zorder=5, label='Estimated Tags')

# Tag Names (MODIFICADO AQUI: fontsize=6)
for i in range(num_tags):
    tag_name = reverse_tag_map.get(i, f"ID_{i}")
    pos = est_tag_pos_aligned[i]
    # Tamanho da fonte reduzido para 6
    plt.text(pos[0] + 0.05, pos[1] + 0.05, tag_name, fontsize=6, color='darkred', weight='bold')

plt.title(f"Visual SLAM Final Result - RMSE: {rmse_aligned:.3f}m")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend(loc='best')
plt.grid(True)
plt.axis('equal')
plt.tight_layout()

print("Saving plot to 'slam_final_graph.png'...")
plt.savefig('slam_final_graph.png', dpi=300, bbox_inches='tight')
print("Plot saved.")


# ==========================================
# 10. EXPORT YAML (COMPATIBLE WITH ROBOT CONTROL)
# ==========================================
print("\nSaving 'dataset_otimizado.yaml' with dataset structure...")

yaml_output = {
    'map_tags': [],         
    'frames': []  # Same structure as input dataset for robot compatibility
}

# 1. Save Optimized Tags (Reference)
for i in range(num_tags):
    name = reverse_tag_map.get(i)
    pos = est_tag_pos_aligned[i]
    yaml_output['map_tags'].append({
        'tag_name': name,
        'position': {
            'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2])
        }
    })

# 2. Save Optimized Trajectory (Preserving Timestamps)
for i in range(num_frames):
    pos = aligned_cam_pos[i]
    
    # Calculate Optimized Rotation aligned to World
    r_vec_local = est_cam_rot_opt[i]
    r_obj_local = R.from_rotvec(r_vec_local)
    r_final = R_align_obj * r_obj_local 
    qx, qy, qz, qw = r_final.as_quat()
    
    # Get original timestamp
    original_ts = frames[i].get('timestamp', 0.0)
    
    # Structure: timestamp + pose (replacing ground_truth with optimized)
    frame_entry = {
        'timestamp': original_ts,
        'pose': {  # Or 'ground_truth_pose' if your robot strictly requires that key name
            'position': {'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2])},
            'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
        }
    }
    yaml_output['frames'].append(frame_entry)

with open('dataset_otimizado.yaml', 'w') as f:
    yaml.dump(yaml_output, f, default_flow_style=False, sort_keys=False)

print("File 'dataset_otimizado.yaml' saved successfully (Robot-Ready)!")

# ==========================================
# 11. TAG ERROR ANALYSIS
# ==========================================
print("\n" + "="*70)
print(f"{'TAG ESTIMATION ERROR ANALYSIS':^70}")
print("="*70)

gt_tags_file = 's_curve_dense_tags_scenario.yaml'
try:
    with open(gt_tags_file, 'r') as f:
        gt_scenario_data = yaml.safe_load(f)
    
    gt_tags_map = {}
    
    # Handle different YAML structures
    if 'tags' in gt_scenario_data:
        if isinstance(gt_scenario_data['tags'], dict):
            for t_name, t_data in gt_scenario_data['tags'].items():
                p = t_data['position']
                gt_tags_map[t_name] = np.array([p['x'], p['y'], p['z']])
        elif isinstance(gt_scenario_data['tags'], list):
             for t_item in gt_scenario_data['tags']:
                t_name = t_item.get('tag_name') 
                p = t_item['position']
                if t_name:
                    gt_tags_map[t_name] = np.array([p['x'], p['y'], p['z']])
    else:
        # Fallback for flat structure
        for t_name, t_data in gt_scenario_data.items():
            if isinstance(t_data, dict) and 'position' in t_data:
                p = t_data['position']
                gt_tags_map[t_name] = np.array([p['x'], p['y'], p['z']])

    # Print Table
    print(f"{'Tag Name':<15} | {'Est (x,y,z)':<22} | {'True (x,y,z)':<22} | {'Error (m)':<10}")
    print("-" * 75)
    
    total_tag_err = 0.0
    count_tags = 0
    
    for i in range(num_tags):
        t_name = reverse_tag_map.get(i)
        est_pos = est_tag_pos_aligned[i]
        
        if t_name in gt_tags_map:
            true_pos = gt_tags_map[t_name]
            err = np.linalg.norm(est_pos - true_pos)
            total_tag_err += err
            count_tags += 1
            
            est_str = f"{est_pos[0]:.2f},{est_pos[1]:.2f},{est_pos[2]:.2f}"
            true_str = f"{true_pos[0]:.2f},{true_pos[1]:.2f},{true_pos[2]:.2f}"
            
            print(f"{t_name:<15} | {est_str:<22} | {true_str:<22} | {err:.4f}")
        else:
            print(f"{t_name:<15} | Not found in GT Scenario file")

    if count_tags > 0:
        avg_tag_err = total_tag_err / count_tags
        print("-" * 75)
        print(f"Average Tag Position Error: {avg_tag_err:.4f} m")

except FileNotFoundError:
    print(f"File '{gt_tags_file}' not found. Cannot compare tag errors.")
except Exception as e:
    print(f"Error reading GT scenario: {e}")

print("="*70)
plt.show()
