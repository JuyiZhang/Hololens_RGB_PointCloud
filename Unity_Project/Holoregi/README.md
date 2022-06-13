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
