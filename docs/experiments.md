# Experiments & Results

## 1. Experimental Setup
- **Simulation:** Gazebo (track.world)  
- **Robot:** AgileX LIMO  
- **Camera:** Realsense D435 (simulated)  
- **ROS 2 Version:** Humble Hawksbill  
- **Testing Environment:** Ubuntu 22.04  

---

## 2. Scenarios
| Test | Description | Expected Outcome |
|------|--------------|------------------|
| **T1** | Teach-run around full track | All tags logged with correct poses |
| **T2** | Autonomous replay | Robot follows route with ≤ 10 cm lateral error |
| **T3** | Relocalization mid-route | Robot resumes trajectory near correct tag |
| **T4** | Tag occlusion | Robot continues using odometry until next tag |

---

## 3. Metrics
| Metric | Description | Target |
|---------|-------------|--------|
| **Pose Error (m)** | Distance between recorded and replayed positions | ≤ 0.10 m |
| **Heading Error (°)** | Angular deviation from path | ≤ 5° |
| **Resume Latency (s)** | Delay after detecting new tag | ≤ 2 s |
| **Detection Rate (%)** | Detected tags vs total visible | ≥ 90 % |

---

## 4. Results Summary
*(Add plots and screenshots here)*  
- Performance graphs: error vs. distance  
- Screenshot of Gazebo replay  
- RViz visualization of tag detections  

---

## 5. Discussion
Analyze:
- Influence of tag spacing on accuracy  
- Effect of velocity on detection rate  
- Simulation-to-real transfer results  

---

## 6. Conclusion
Summarize the achieved autonomy level and the next improvements:
- Fine-tune PID gains  
- Improve detection filtering  
- Implement loop closure in future work
