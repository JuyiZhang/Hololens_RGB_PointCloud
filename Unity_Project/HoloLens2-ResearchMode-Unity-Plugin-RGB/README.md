# HoloLens2-ResearchMode-Unity-RGB Version
This plugin is modified based on the Hololens2-ResearchMode-Unity Repository (Please see the reference repository or the link [Hololens2-ResearchMode-Unity](https://github.com/petergu684/HoloLens2-ResearchMode-Unity)

## Main Modification:
Added Video Frame Processor from [Hololens2ForCV](https://github.com/microsoft/HoloLens2ForCV) <br>
The original Video Frame Processor is created to write the data onto Hololens' disk. However, since we want it to be imported to Unity, I have adapted it to write video frame buffer to the `HL2ResearchMode` Class <br>
Modified `HL2ResearchMode.cpp`/`HL2ResearchMode.h` to reflect the data retrieval from Video Frame Processor
## Bugs
- At version 1432. There is memory leakage when accessing PV camera and the Short Throw Sensor at the same time. So don't be surprised if it crashed
- At version 1501. It will not work because microsoft "Fixed a bug that causes the leakage when accessing the PV Camera and Depth Sensor at the same time"  (Thanks, Microsoft)

## Original Content from Peter Gu
Unity Plugin for using research mode functionality in HoloLens 2. Modified based on [HoloLens2ForCV](https://github.com/microsoft/HoloLens2ForCV).

![Depth Map Example](https://github.com/petergu684/HoloLens2-ResearchMode-Unity/blob/master/DepthMapExample.jpg)

Skeleton to wrap HoloLens 2 research mode api into Windows Runtime extension. 

To use it in Unity,
- Build this project (ARM64,Release) and copy the .dll and .winmd files in `HL2UnityPlugin\ARM64\Release\HL2UnityPlugin` into `Assets/Plugins/WSA/ARM64` folder of your Unity project.
- Change the architecture in your Unity build settings to be ARM64.
- After building the visual studio solution from Unity, go to `App/[Project name]/Package.appxmanifest` and add the restricted capability to the manifest file. (Same as what you would do to enable research mode on HoloLens 1, reference: http://akihiro-document.azurewebsites.net/post/hololens_researchmode2/)
```xml 
<Package 
  xmlns:mp="http://schemas.microsoft.com/appx/2014/phone/manifest" 
  xmlns:uap="http://schemas.microsoft.com/appx/manifest/uap/windows10" 
  xmlns:uap2="http://schemas.microsoft.com/appx/manifest/uap/windows10/2" 
  xmlns:uap3="http://schemas.microsoft.com/appx/manifest/uap/windows10/3" 
  xmlns:uap4="http://schemas.microsoft.com/appx/manifest/uap/windows10/4" 
  xmlns:iot="http://schemas.microsoft.com/appx/manifest/iot/windows10" 
  xmlns:mobile="http://schemas.microsoft.com/appx/manifest/mobile/windows10" 
  xmlns:rescap="http://schemas.microsoft.com/appx/manifest/foundation/windows10/restrictedcapabilities" 
  IgnorableNamespaces="uap uap2 uap3 uap4 mp mobile iot rescap" 
  xmlns="http://schemas.microsoft.com/appx/manifest/foundation/windows10"> 
```

```xml
  <Capabilities>
    <rescap:Capability Name="perceptionSensorsExperimental" />
    <Capability Name="internetClient" />
    <Capability Name="internetClientServer" />
    <Capability Name="privateNetworkClientServer" />
    <uap2:Capability Name="spatialPerception" />
    <DeviceCapability Name="backgroundSpatialPerception"/>
    <DeviceCapability Name="webcam" />
  </Capabilities>
```
`<DeviceCapability Name="backgroundSpatialPerception"/>` is only necessary if you use IMU sensor. 
- Save the changes and deploy the solution to your HoloLens 2.


## Note:
- The reconstructed point cloud still has the offset problem as is described [here](https://github.com/microsoft/HoloLens2ForCV/issues/12) for object beyond 1m.
- To visualize the depth image, you need a grayscale shader applied to your preview plane. Example: [grayscale shader](https://github.com/qian256/HoloLensARToolKit/blob/master/HoloLensARToolKit/Assets/Sample/Grayscale.shader).
- For point cloud, current implementation only returns the reconstructed point cloud as a float array (in the format of x,y,z,x,y,z,...). If you want to visualize it, I find [this project](https://github.com/MarekKowalski/LiveScan3D-Hololens) is a good example.
- This project is mainly to show how to use Reseach Mode in Unity. I only provided implementation on AHAT camera image visualization and point cloud reconstruction (based on depth map of AHAT camera), two front spatial camera. The long-throw depth sensor and IMU sensor are also available thanks to @HoloAdventure. Feel free to modify the code according to your own need.
- Only one of the short-throw(AHAT) and long-throw depth sensor should be enabled at the same time.
- If you need a sample project to get started, you can refer to UnitySample folder.
