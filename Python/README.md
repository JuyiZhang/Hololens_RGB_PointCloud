# Python Codes
This is the place where all the python codes are located. The detailed usage instruction is shown below
## TCPServer.py
### Usage
The python server takes sensor data from Hololens2 and computes registration results. The resulted 
transformation matrix is sent back to Hololens2 for result visualization.
### Instructions

#### Set the IP Address and Port
The IP address is the machine IP address shown in the network tab (If you are using home router, it is usually `192.168.X.X`. If it is a remote server, use public IP address<br>
The port can be any number in 0-65535, but make sure not to use some reserved port or conflit with other port (Some example would be port 1024)

#### Prepare the 3D holograms
prepare the medical holograms in the root directory, `./PhantomHead.stl`,`./PatientMRI.stl`
`./PatientCT.stl`. 

#### Modify the Saving Location
By default, the point cloud data from Hololens2 is saved in `./data/PointCloudCapture`.
##### That's it. Run it and Enjoy!


[//]: # (## main.py)

[//]: # ()
[//]: # (### Usage)

[//]: # (The file is used to perform registration between Hololens frames and the MRI mesh.)

[//]: # ()
[//]: # (### Instruction)

[//]: # (#### Data preparation)

[//]: # (Reconstruct RGB point clouds from `ImageCapture` and `PointCloudCapture` folders by following the instruction from HoloLens2ForCV:)

[//]: # (https://github.com/microsoft/HoloLens2ForCV/tree/main/Samples/StreamRecorder <br>)

[//]: # (Then pick about 10 best frames that cover the entire object. Mannuly crop them to about the same range of the MRI file using MeshLab: https://www.meshlab.net/, and put them into the `cropped_pcds` folder. Use MeshLab also to generate the MRI point clouds from the MRI mesh file. The MRI point cloud and Hololens frames are already provided here for example. )

[//]: # (#### Registration)

[//]: # (Simplly run `python main.py` to start registration that is described in the report. Two different kinds of registration techniques &#40;`global_registration.py` and `multiway_registration`&#41; are also provided.)

[//]: # ()
[//]: # (#### Denoise the raw noisy point cloud)

[//]: # (Run `denoising.py`, the sample data are provided)

[//]: # ()
[//]: # (#### Evaluation registration)

[//]: # (Run  'evaluation.py`, the sample data are provided)

[//]: # ()
[//]: # (#### Detect keypoints in point clouds)

[//]: # (Run `keypoint_detection.py`, the sample data are provided)

[//]: # ()
[//]: # (#### Denoise the raw noisy point cloud)

[//]: # (run `registration_robust_experiment.py`, the sample data are provided)

[//]: # ()
[//]: # ()
[//]: # ()
[//]: # (#### Output)

[//]: # (`main.py` generates the transformed MRI point cloud and the combined hologram registered with MRI separately. )
