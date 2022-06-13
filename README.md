![alt text](Holoregilogo.png "Holoregi")
## Welcome to the Holoregi Repository
This is a repository created for the Computer Vision Project of "Patient Registration in Hololens 2" Sponsored by Augmedit
## What is Holoregi
Holoregi is the abbreviated version of Hololens Registration, which utilizes the Hololens 2 research mode to obtain spatial information. The spatial information is then send to an computer and perform registration, which generates a transformation matrix and the transformation matrix will be sent back to Hololens and display the MRI scan at the transformed location
## Environment
### Server Side
- Operating System: macOS (Arm/x64), Linux (Arm/x64), Windows (x64)
- Python: 3.9 and below
- Package: Open3D 0.15.1, Scipy 1.8.0, Sklearn 1.0.2, Numpy 1.22.4
### Client Side
- Operating System (Compile): Windows (x64)
- Operating System (Hololens): Windows, build number 20348.1432 (Note that current version 22H2 (build number 20348.1501 is **NOT COMPATIBLE** due to microsoft introducing some bugs for accessing AHAT sensor and the PV camera at the same time)
- Unity: 2020.3.30f1
- Additional Package: MRTK 2.8.0, OpenXR 1.3
## How to Run
### Server Side
Reconstruct RGB point clouds from ImageCapture and PointCloudCapture folders by following the instruction from HoloLens2ForCV: https://github.com/microsoft/HoloLens2ForCV/tree/main/Samples/StreamRecorder <br>
Then process those point clouds described in the `python` folder. With the provided MRI point cloud file, run <br>
`python main.py` <br>
to get the transformed MRI point cloud and combined hologram from the Hololens frames registerd with it.
### Client Side
#### Compiling and Modifying
Open the Project in Unity. The build setting should already been up <br>
The build setting is already setup, after successful build, please copy the package.appxmanifest in root directory to `App/HL2ResearchModeUnitySample` <br>
For more info, please refer to the Unity Folder of Holoregi <br>

