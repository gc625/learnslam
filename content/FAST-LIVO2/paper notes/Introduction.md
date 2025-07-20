# Introduction
The introduction can be distilled into four sections:

## 1. The Significance of SLAM: 
Introduce application examples and highlight the potential of SLAM.

## 2. Issues in SLAM (Motivation for FAST-LIVO2): 
Begin by discussing single-sensor limitations, leading to multi-sensor integration needs, and ultimately introducing FAST-LIVO2.
- **Single-sensor SLAM:**
    - **a. Visual SLAM:** Rich texture information and strong potential for scene understanding; however, it lacks depth information and struggles with textureless areas, uneven illumination, and noise.
    - **b. LiDAR SLAM:** Provides precise depth information but lacks texture information and struggles with structured environments (e.g., tunnels or planar walls).
- **Necessity for Multi-sensor SLAM (LiDAR-Inertial-Visual SLAM or LIVO):**  
    Achieves more accurate state estimation, denser and texture-rich 3D maps, and greater environmental adaptability—even if individual sensors degrade.
- **Current challenges faced by LIVO:**
    - **a.** Low computational efficiency due to massive point clouds within limited onboard computing resources.
    - **b.** Feature extraction-based methods reduce computational load but involve extensive engineering tricks and struggle in textureless or structureless environments.
    - **c.** Unified map management can significantly improve system efficiency, but designing a data structure that effectively integrates heterogeneous measurements from LiDAR and cameras is challenging.
    - **d.** Generating precise texture maps requires pixel-level accuracy, demanding highly accurate hardware synchronization, precise sensor extrinsics, sensor noise analysis, and efficient algorithmic design.
## 3. The Concept and Significance of FAST-LIVO2:  
The aforementioned points form both the problems and motivation. FAST-LIVO2 uses an Error-State Iterated Kalman Filter (ESIKF) to sequentially update states for LiDAR and visual modules:
- IMU measurements are used for state prediction in ESIKF, providing a prior state. 
- Based on this prior, LiDAR point clouds are undistorted through back-propagation, computing the LiDAR measurement equations (plane-to-point distances). The state is iteratively updated through ESIKF, followed by updates to a voxel octree map (plane priors), obtaining a quasi-posterior state.
- Using this quasi-posterior state, LiDAR points serve as visual map points. The photometric errors between reference image patches and the projection of visual map points onto the current image are computed, followed by ESIKF iterative updates, exposure time estimation, and updating reference patches, ultimately obtaining the posterior state.
Additionally, FAST-LIVO2 employs on-demand voxel ray-casting (note: detailed explanation follows later) to address LiDAR blind spots.
**Contributions:**
1. Proposes a sequential-update ESIKF framework integrating heterogeneous measurements, improving system robustness.
2. Refines LiDAR-derived plane priors (plane parameters) to enhance precision, avoiding assumptions that image patches have uniform depth (note: if unclear, the core difference lies in whether an affine transform is computed based on plane parameters; detailed explanations can be found in code analysis).
3. Introduces a reference-patch updating strategy, improving image alignment accuracy.
4. Estimates camera exposure time online to handle varying textures.
5. Uses on-demand voxel ray-casting to enhance robustness in areas lacking LiDAR points.


Next section: [Related Works](<Related Works>)