# Holoregi Unity Project
Welcome to the Holoregi Unity Project. Before putting the project into your Hololens. Here are some must-knows that we recommend from reading
## Platform Support
- Only `Windows x86/64` is supported. With great regret microsoft does not support compiling the App on macOS and Linux (I have not personally tested on a AMD processor, and the M1 Max Simulated Windows 11 will **NOT** work
- The Visual Studio Version I used is `2019`. However, newer version should be okay
- The Unity Version is `2020.3.30f1` and the MRTK Version is `2.8.0`. Later version might be supported until microsoft decides to change the API and desert every developer like what they have done in Windows Phone 8
## How to compile
Open the project in Unity, Check that the Build Settings are Universal Windows Platform. 
- The target device should be `Hololens`
- The Architecture should be `ARM64`
- The Target SDK Version and Visual Studio Version should be `Latest Installed`. 
- Use `Release` as build configuration
- Add the scene in the scene folder
- Remember to change the IP Address in the Scene. It is an monoscript instance called TCP Server in `MixedRealityPlayspace/ResearchModeController` game object. The IP Address and the port should be the same as your python `TCPServer.py` file specifies
Click build, and assign build folder (Note: There are filesystem issue with windows and sometimes it says cannot copy file to some destination. Shutting down Visual Studio should resolve the issue)
## How to run
- Copy the Package.appxmanifest to `[Your Build Folder]/Holoregi` and substitute the old file. It is important for Research Mode Access
- Please Turn on the Research Mode when running the App
- During First bootup. The App might asks for camera and microphone access. Click `Yes`
- For some mysterious reason. After granting the access, you will need to restart the Hololens for it to work (If a soft restart says `Update and restart`. **DO NOT DO THAT** It is a long story of someone restarted the hololens, found that the system got upgraded to 1501 and the app crashes. It will take forever to downgrade the device, if it is still possible)
## How to use
- After entering the app. You can see seven buttons, a big chunk of head, and a sphere
- The Calibrate Button will not work
- Toggle point cloud will toggle the display of point cloud
- Open the server on your computer
- Click Connect to server button, the circle on the left should turn green
- Click Enable Clipping to Toggle the Sphere On and Off
- If you click hide menu, you can yell "Menu" to the air to recall it (and do not do it in public unless you are comfortable with being "the unique one")
- Click Send Point Cloud to begin sending the point cloud
- If you implemented sending the transformation matrix, you can click toggle registration and see the head flying around the sky.
## How to code
There is detailed comment in the code on the purpose. All you need is basic C# knowledge and you can achieve a lot of things (Hopefully one day unity or some other game engine will dump C# in favor of python)
