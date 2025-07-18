


# Abstract 
The author condensed the abstract into the following 4 aspects:
## **Q1: What is Fast-LIVO2?**
**FAST-LIVO2** is a real-time, accurate, and robust tightly coupled multi-sensor SLAM system. It effectively integrates heterogeneous sensor data from lidar, inertial, and visual sensors using the Error-State Iterated Kalman Filter (ESIKF).  
(Note: The lidar sensors include repetitive scanning multi-line rotating lidars such as Velodyne-16, XT-32, Ouster-64, Pandar128, and non-repetitive scanning solid-state lidars such as Livox-Avia. Cameras include pinhole cameras and fisheye cameras.)

## **Q2: What does FAST-LIVO2 solve?**

1. Heterogeneity of measurement data between LIDAR and camera; 
2. System Efficiency 
3. System accuracy
4. System Robustness

## **Q3: What does FAST-LIVO2 do?**
1. Employ a sequential update strategy (first running LIO then running VIO based on the state updated by LIO) to effectively fuse heterogenous measurements from LiDAR and camera sensors into a unified system.
2. It Directly registers raw data, avoiding complex and time consuming feature extraction processes. Measurements from both LiDAR and camera sensors are integrated into a unified voxel map (octree), enabling efficient management of 3D data.
3. Establishes associations between LiDAR points and image patches to achieve accurate image alignment. Additionally, it dynamically refines plane parameters corresponding to LiDAR points, further enhancing alignment accuracy.
4. Employs on demand voxel ray casting to overcome situations where data lies within LiDAR blind spots. Additionally camera eposure times are estimated online to address timestamp offsets caused by variable exposure durations under uneven illumination conditions. 

## Q4: What has FAST-LIVO2 demonstrated?
1. **Real-time performance (efficiency):** Demonstrated through UAV navigation experiments.
2. **Accuracy and robustness:** Proven by UAV-based 3D reconstruction. 
3. **Scalability:** Output results can be directly utilized for rendering methods like Neural Radiance Fields (NeRF) or 3D Gaussian Splatting (3D-GS).
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

# Related Work

The related work can be summarized into three parts:
**1. Overview of Direct Methods in SLAM**  
**2. Introduction to LIVO**  
**3. Development Timeline of LIVO (Note: The timeline is based solely on works mentioned in this paper. Given the author's limitations, some relevant work may inevitably be overlooked.)**
## 1. Overview of Direct Methods

A comparative analysis between feature-based and direct methods is presented.
**Classification of Visual Direct Methods:**
- **Dense Direct Methods (all pixels):**  
    Typically applied to RGB-D cameras. Examples include:
    - _"Real-time dense visual tracking under large lighting variations"_
    - _"Direct iterative closest point for real-time visual odometry"_
    - _"Robust odometry estimation for RGB-D cameras"_
- **Semi-dense Direct Methods (pixels with significant grayscale gradients):**  
    Examples include:
    - _"LSD-SLAM: Large-scale direct monocular SLAM"_
    - _"Semi-dense visual odometry for a monocular camera"_
- **Sparse Direct Methods (image patches):**  
    Based on carefully selected image patches. Examples include:
    - _"Direct sparse odometry"_
    - _"SVO: Fast semi-direct monocular visual odometry"_

**Direct Methods in LiDAR:**  
There is no explicit categorization. Typically involves downsampling scans and establishing point-to-plane constraints. Representative works include:
- _"FAST-LIO2: Fast direct lidar-inertial odometry"_
- _"Efficient and probabilistic adaptive voxel mapping for accurate online lidar odometry"_
- _"Direct lidar-inertial odometry: Lightweight LIO with continuous-time motion correction"_
- _"D-LIOM: Tightly-coupled direct lidar-inertial odometry and mapping"_
**Direct Methods in FAST-LIVO2:**
- For the LiDAR module: utilizes methods from _"Efficient and probabilistic adaptive voxel mapping for accurate online lidar odometry"_ (Note: manages voxel maps using hash tables and octrees).
- For the visual module: adopts _"SVO: Fast semi-direct monocular visual odometry"_ (Note: employs a sparse direct method to compute the Jacobian matrix of photometric error relative to pose perturbations. FAST-LIVO2 differs from SVO in terms of coordinate systems and thus requires an additional processing step—details available in code analysis).

## 2. Introduction to Multi-sensor Fusion SLAM

(Note: This section is highly recommended for quickly grasping relevant LIVO research.)

The authors classify existing LIVO methods into two categories: loosely-coupled and tightly-coupled. The classification is mainly based on two criteria:

- **State Estimation:**  
    Whether the estimation result from one sensor is included as an optimization target within another sensor's model.
    
- **Raw Measurement Integration:**  
    Whether raw data from different sensors are directly combined.
    


## 3. Development Timeline of LIVO

First, according to the classification standards in Section 2, all existing LIVO methods are categorized as shown below:
![[fig1.png]]
**Figure 1:** LIVO Development Timeline 

**Loosely-coupled Methods:** Discussed from the aspects of states and measurements, illustrated as follows:
![[fig2.png]]
**Figure 2:** Timeline of Loosely-coupled Methods (not included here, referenced from the original text).

**Tightly-coupled Methods:** The paper discusses these methods from two perspectives: indirect methods and direct methods.
![[fig3.png]]

# System Overview

This section primarily describes the processing flow of the entire system.

Figure 4 illustrates the system architecture, showing that the system consists of four core modules:

1. Sequential-update ESIKF
2. Local map construction
3. LiDAR observation model
4. Visual observation model

Specifically, Module 1 deals with the ESIKF's state prediction/update equations, while Modules 3 and 4 define the state observation equations of the ESIKF.

The processing flow is as follows:
LiDAR points sampled asynchronously are merged to align with the camera's sampling timestamps → Sequential updates are performed using ESIKF (first LiDAR, then camera). Both sensor modalities utilize a voxel map for management.

The LiDAR observation model performs frame-to-map point-to-plane residual calculations.

Visual map points within the current Field of View (FoV) are extracted from the voxel map using visible voxel queries and on-demand ray casting. Photometric errors (frame-to-map residuals, meaning the variation of photometric error between current and reference frames under pose perturbation $^GT_I$) are computed, enabling visual updates.

Local map maintenance: LiDAR-Inertial Odometry (LIO) updates geometric information of the voxel map, whereas Visual-Inertial Odometry (VIO) updates the current and reference image patches associated with the visual map points. (Updated reference image patches are further refined to obtain their normals in a separate thread.)

## Sequential-update **ESIKF**
f readers are already familiar with ESIKF or the FAST-LIO system, this section can be skipped. However, for those unfamiliar, this section provides a step-by-step introduction to ESIKF.

This section introduces the mathematical models for state prediction and update, structured into four subsections:
S
1. **Notation and State Transition Model**
2. **Scan Merging**
3. **Propagation (Forward and Backward Propagation)**
4. **Sequential Updates**


### Notation and State Transition Model
First, sensor coordinate frames are defined:

$$
\begin{aligned} 
I : \text{IMU frame} \\ 
G : \text{Global frame} 
\end{aligned}
$$

(Note: Only IMU and global frames are defined here since the entire system state is IMU-centric. Observations from LiDAR and camera are transformed into the IMU frame.)
Next, the discrete-time state transition equation is given by:
$$x_{i+1} = x_i \boxplus (\Delta t f(x_i, u_i, w_i))$$
- $x_i$ is the state vector
- $u_i$ represents IMU observations
- $w_i$ is the process noise
- $\Delta t$ is the IMU sampling interval
- $f$ is the kinematic equation matrix
(Note: When deriving the state-space equations, some methods begin by differentiating the kinematic equations in continuous time to obtain continuous-time state-space equations, which are then discretized. Discretization typically involves either Euler integration or midpoint integration. This process can be tedious, as continuous-time equations must first be obtained. A more convenient alternative is to directly discretize the kinematic equations and derive discrete-time state-space equations directly. Readers are encouraged to refer to additional [blogs](https://cc1924.blog.csdn.net/article/details/129425086?spm=1001.2014.3001.5502) for further details on state-space equation derivations.)

**Explanation of the $\boxplus$ operator:**  
Readers who have only read the FAST-LIVO2 paper may wonder what the $\boxplus$ symbol represents. In fact, it's not complicated. This operator is explained clearly in the paper _"FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter."_ The operator arises primarily because rotation matrices $R\in SO(3)$ do not form a closed group under standard addition. Simply put, consider a coordinate frame F undergoing rotations $R^0_{1}$​ followed by $R^1_2$​, resulting in rotation $R^0_2$​. In this case, the relationship is $R^0_2\neq R^0_1 + R^1_2$​, but rather $R^0_2=R^0_1R^1_2$​. Thus, the standard addition operator (+) becomes inappropriate for representing rotation increments. Hence, the $\boxplus$ operator is introduced to suitably represent "addition" operations involving both the rotation space $SO(3)$ and the Euclidean space $\mathbb{R}^n$.  
(Note: This explanation is a simplified interpretation. To thoroughly understand addition and subtraction operations in $SO(3)$, readers are encouraged to further explore concepts related to manifolds and Rodrigues' formula in sources like _"Autonomous Driving and SLAM Technology in Robotics"_ or _"14 Lectures on Visual SLAM"_ by Gao Xiang.)
