# Detailed Documentation of Stereo Vision and Robot Trajectory Planning Software

## Part 1: Stereo Image Processing (`stereo_main.py`)

### Introduction
`stereo_main.py` is the central script of the system, handling stereo image processing to generate depth information and plan the robot's trajectory based on this data. The script uses OpenCV for image processing and implements stereo vision algorithms along with pathfinding techniques.

### Setup and Global Variables
The script begins by setting up various paths to image streams and calibration files. Key global variables and parameters for stereo vision processing are defined:

- **Paths** (`PATH_L`, `PATH_R`, `PATH_CALIB`): Directories for left and right image streams and calibration files.
- **Stereo Matcher Parameters**: Includes minimum disparity (`minDisp`), number of disparities (`nDisp`), block size (`bSize`), and parameters `P1` and `P2` for the block matching algorithm.
- **Weighted Least Squares (WLS) Parameters**: Regularization (`lam`) and contrast sensitivity (`sigma`) parameters for the WLS filter.
- **Calibration Parameters Loading**: Undistortion and rectification maps for left and right cameras, along with the Q matrix necessary for converting disparity to depth.

### Main Function
The `main` function is the entry point of the script. It performs the following operations:

1. **Frame Processing**: Processes a range of stereo image frames. For each frame, it computes the disparity and subsequently the depth map.

2. **Real-Time Visualization**: The script can be configured to either process a stream of images in real-time or work on a single image pair.

3. **Disparity Computation and Depth Estimation**: Calls `compute_disparity` to process each image pair to compute disparity and depth maps.

### Disparity Computation (`compute_disparity`)
This function performs the following steps:

1. **Image Loading and Rectification**: Reads the stereo images, applies the calibration data to rectify them, ensuring that they are aligned and undistorted.

2. **Grayscale Conversion**: Converts the images to grayscale, as the disparity computation in OpenCV works on single-channel images.

3. **StereoSGBM Matcher**: Initializes the Stereo Semi-Global Block Matching (SGBM) matcher with the predefined parameters. This matcher computes disparity by comparing blocks of pixels between the two images.

4. **WLS Filter Application**: Applies the WLS filter to the raw disparity map. This step smooths the disparity and reduces noise, enhancing the quality of the depth estimation.

5. **3D Point Cloud Generation**: Converts the disparity map to a 3D point cloud using the Q matrix. This step is crucial for depth estimation and understanding the spatial arrangement of objects in the scene.

### Path Planning (`find_path`)
After obtaining the depth information, the script proceeds to plan the robot's trajectory:

1. **Obstacle Identification**: Uses the depth information to identify obstacles in the robot's environment.

2. **Occupancy Grid Creation**: Transforms the obstacle data into an occupancy grid, a 2D representation of the environment where each cell indicates whether the space is occupied (by an obstacle) or free.

3. **A* Pathfinding Algorithm**: Implements the A* algorithm to find the shortest path from the robot's current position to the target location, avoiding obstacles. The algorithm considers the occupancy grid and computes the path based on the cost of traversing each cell.

4. **Real-Time Path Adjustment**: In a real-time scenario, this step would involve continuously updating the occupancy grid and recalculating the path as new depth data becomes available.

### Visualization and Output
The script includes functionality to visualize the results:

1. **Disparity Maps Display**: Shows the raw and WLS-filtered disparity maps for analysis and debugging.

2. **Path Visualization**: Plots the planned path on the occupancy grid and potentially in a 3D representation of the environment.

3. **Debugging Information**: Displays various parameters and statistics like computation time, path length, etc., for performance monitoring and debugging.

## Part 2: In-depth Analysis of Algorithms in `stereo_main.py`

### 1. Stereo Semi-Global Block Matching (SGBM) Algorithm

#### Overview
StereoSGBM (Semi-Global Block Matching) is a key algorithm in stereo vision used for computing disparity maps. It is an extension of the basic Block Matching algorithm, enhanced with global optimization techniques.

#### How it Works
- **Block Matching**: The basic idea is to find the correspondence between blocks of pixels in the left and right images. For each block in the left image, the algorithm searches within a defined range in the right image to find the best match based on some similarity measure.
- **Semi-Global Optimization**: Unlike simple block matching, SGBM applies smoothness constraints not only in horizontal directions but also along several other paths (e.g., vertically and diagonally). This multi-directional approach helps in achieving more accurate and globally consistent disparity values.
- **Cost Aggregation**: SGBM aggregates matching costs along these paths and uses dynamic programming for optimization. This process reduces the effects of noise and occlusions.
- **Parameters**: 
  - `minDisparity`: Starting point of the disparity search.
  - `numDisparities`: The range of disparities to be checked.
  - `blockSize`: Size of the block for matching.
  - `P1`, `P2`: Parameters controlling the smoothness constraint. Larger values enforce smoother disparities but may oversmooth depth discontinuities.
  
### 2. Weighted Least Squares (WLS) Filter

#### Overview
The WLS filter is used to refine the disparity map obtained from SGBM, particularly to smooth out disparities while preserving edges.

#### How it Works
- **Edge-Preserving Smoothing**: The WLS filter applies more smoothing to regions with low contrast (i.e., less likely to have depth discontinuities) and less smoothing to high-contrast edges (more likely to be depth discontinuities).
- **Parameters**:
  - `Lambda (λ)`: Regularization parameter. A higher value gives more smoothing.
  - `Sigma`: Controls sensitivity to edges. A higher sigma means that more pixels will influence each other, leading to more aggressive smoothing.

### 3. 3D Point Cloud Generation

#### Concept
This process involves converting the disparity map into a 3D point cloud, which represents the depth of each pixel.

#### Mathematics
- The Q matrix, obtained from stereo calibration, is used here. It encodes the camera's extrinsic (i.e., relative position and orientation) and intrinsic (focal length, principal point, etc.) parameters.
- Each pixel in the disparity map is transformed into 3D space using the Q matrix, resulting in a point cloud where each point represents the 3D coordinates of the corresponding pixel.

### 4. A* Pathfinding Algorithm

#### Overview
A* (A-star) is a widely-used pathfinding and graph traversal algorithm, which is efficient and guarantees the shortest path under certain conditions.

#### How it Works
- **Heuristic-Based Search**: A* uses a heuristic to estimate the cost from the current node to the goal, in addition to the cost from the start to the current node. This heuristic helps A* to prioritize paths that appear to be leading closer to the goal.
- **Occupancy Grid**: In the context of this software, the occupancy grid represents the robot's environment, where each cell's value indicates whether the space is free or occupied by an obstacle.
- **Path Computation**: A* navigates through the grid, considering both the actual cost to reach each cell (known) and the estimated cost from that cell to the goal (heuristic). This approach efficiently finds the shortest path avoiding obstacles.

## Part 3: Analysis of Image Capturing (`capture_stream.py`)

### Overview
`capture_stream.py` is designed for capturing a continuous stream of images from a Raspberry Pi camera. This script is crucial for real-time stereo vision applications, as it provides the input images necessary for depth estimation and path planning.

### Key Components and Functionality

1. **Initialization and Configuration**: 
   - **Camera Setup**: The script initializes the PiCamera and sets up its resolution and framerate.
   - **Camera Parameters**: Parameters like camera rotation, shutter speed, and white balance (AWB) are configured. Disabling the automatic exposure and AWB allows for consistent image capture under varying lighting conditions.

2. **Button-Triggered Capture**:
   - The script uses GPIOZero to interface with a physical button connected to the Raspberry Pi.
   - Image capture starts when the button is pressed, allowing for synchronized captures in a stereo camera setup.

3. **Image Capture Loop**:
   - The camera continuously captures images in MJPEG format.
   - A custom `SplitFrames` class is used to handle the output of each frame. This class writes each frame to a file when it detects the start of a new JPEG frame in the buffer.
   - The script records the number of frames captured and the duration, providing feedback on the capture performance (e.g., frames per second).

4. **Error Handling**:
   - The script includes exception handling to gracefully manage unexpected interruptions (like a keyboard interrupt) or errors during the capture process.

### Technical Details

1. **PiCamera Library**: 
   - This library is specific to the Raspberry Pi and provides a straightforward interface for camera operations. 
   - It handles low-level camera configuration and image capture, abstracting the complexities of interfacing with the camera hardware.

2. **Image Storage**:
   - Captured images are saved as JPEG files. Each file is named sequentially to maintain the order of capture.
   - The path for saving images is predefined, and a new directory is created if it does not exist.

3. **Frame Handling**:
   - The `SplitFrames` class checks for the start of a new JPEG frame (`0xff\xd8` byte sequence) in the buffer and writes each complete frame to a file.
   - This method ensures that each image file corresponds to a distinct frame from the camera stream.

4. **Synchronization**:
   - Synchronization is crucial in stereo vision to ensure that the left and right images correspond to the same moment in time.
   - The button press mechanism is a simple yet effective way to start the capture simultaneously on two Raspberry Pi devices (each connected to one camera of a stereo pair).

### Conclusion

`capture_stream.py` plays a vital role in the stereo vision system by providing a reliable and synchronized method for capturing image streams from the Raspberry Pi camera. Its integration with the hardware buttons for triggering captures and handling of real-time image data are key aspects of its functionality.

## Part 4: Understanding `capture_calib.py` for Calibration Image Capturing

### Overview
`capture_calib.py` is dedicated to capturing images for stereo camera calibration. Accurate calibration is critical in stereo vision to compute precise depth information. This script facilitates capturing still images under controlled conditions, which are then used to determine the camera's intrinsic and extrinsic parameters.

### Key Components and Functionality

1. **Initialization and Path Setup**:
   - Similar to `capture_stream.py`, this script initializes the PiCamera and configures its basic settings like resolution and rotation.
   - It sets up the directory path for saving calibration images. If the directory doesn't exist, it creates one.

2. **Calibration Image Capture**:
   - Unlike `capture_stream.py`, which captures a continuous stream, `capture_calib.py` is designed to capture individual still images.
   - The script uses a button press (GPIO interface) to trigger the capture of each image. This allows for controlled and deliberate image capturing, essential for calibration purposes.

3. **File Naming and Storage**:
   - Captured images are stored sequentially in the predefined directory. 
   - The script automatically determines the filename for the next image based on existing files, ensuring a unique and sequential naming scheme.

4. **Camera Parameters for Consistent Capture**:
   - The script sets specific camera parameters, including shutter speed and white balance, to ensure consistency across all calibration images. 
   - Disabling automatic adjustments is crucial for calibration to avoid variations due to automatic camera adjustments.

### Technical Details

1. **Calibration Image Requirements**:
   - Calibration typically involves capturing images of a known pattern (like a chessboard) from different angles and positions.
   - The consistency in capture settings ensures that variations in the images are due to the camera's position and orientation, not due to changes in camera settings.

2. **Synchronization in Stereo Setup**:
   - For stereo calibration, it's essential to capture corresponding pairs of images from the left and right cameras simultaneously.
   - The use of a physical button to trigger image capture allows for easy synchronization between two cameras in a stereo setup.

3. **Error Handling and Feedback**:
   - The script provides feedback on successful image capture and includes exception handling for errors and interruptions.
   - This feedback is valuable during the calibration process to ensure that sufficient and correct images are captured.

### Conclusion

`capture_calib.py` is a specialized script for capturing images used in the calibration of stereo cameras. Its functionality ensures that the images are captured under consistent settings and are well-organized for subsequent processing. Accurate calibration is foundational to the success of stereo vision applications, making this script a critical component of the overall system.

## Part 5: Deep Dive into `calibration.py` for Stereo Camera Calibration

### Overview
`calibration.py` performs the critical task of calibrating the stereo camera setup. This process is essential to achieve accurate depth measurements in stereo vision. The script computes intrinsic (individual camera) and extrinsic (relative position and orientation) parameters based on a series of images of a known pattern (typically a chessboard).

### Calibration Process

1. **Image Preparation and Loading**:
   - The script loads a series of stereo image pairs from a specified directory. These images should contain views of a calibration pattern (chessboard) captured from various angles and distances.

2. **Chessboard Corner Detection**:
   - For each image, the script uses OpenCV functions to detect the corners of the chessboard pattern. This process involves converting images to grayscale and then finding the chessboard corners using `cv.findChessboardCorners`.
   - Subpixel accuracy is achieved using `cv.cornerSubPix`, which refines the corner locations.

3. **Object Points and Image Points Collection**:
   - Object points are the 3D coordinates of the chessboard corners in a predefined calibration pattern coordinate system.
   - Image points are the 2D coordinates of these corners in the image.
   - The script collects these points for all the calibration images.

4. **Intrinsic Parameters Calibration**:
   - Using the collected object and image points, the script computes the camera matrix and distortion coefficients for each camera independently using `cv.calibrateCamera`.
   - These parameters are essential for correcting lens distortion in the images.

5. **Stereo Calibration**:
   - The script then performs stereo calibration using `cv.stereoCalibrate`. This step computes the rotation and translation between the two cameras, essential for depth calculation.
   - It also refines the individual camera matrices and distortion coefficients in the context of the stereo setup.

6. **Stereo Rectification**:
   - After calibration, `cv.stereoRectify` is used to compute rectification transforms, projection matrices, and the Q matrix.
   - Rectification transforms are used to align the images from both cameras, making them suitable for disparity calculation.
   - The Q matrix is vital for converting disparity values into actual depth measurements.

### Critical Output Parameters

- **Camera Matrices (CL, CR)**: Intrinsic parameters of the left and right cameras.
- **Distortion Coefficients (DL, DR)**: Lens distortion parameters for the left and right cameras.
- **Rotation (R) and Translation (T) Matrices**: Extrinsic parameters describing the relative position and orientation of the two cameras.
- **Rectification Transforms (RL, RR) and Projection Matrices (PL, PR)**: Used for image rectification in stereo processing.
- **Q Matrix**: Essential for converting disparities into 3D points (depth information).

### Visualization and Debugging

- The script includes functionality to visualize the rectified images and the effect of calibration.
- Debugging information like reprojection errors is printed, allowing assessment of calibration quality.

### Conclusion

`calibration.py` is a vital component of the stereo vision system, ensuring that the camera setup is accurately calibrated for precise depth estimation. The process involves intricate image processing and mathematical computations, the accuracy of which directly impacts the performance of the entire stereo vision application.

