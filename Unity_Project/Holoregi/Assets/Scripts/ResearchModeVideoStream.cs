using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using UnityEngine.Windows.WebCam;
using System.Linq;

#if ENABLE_WINMD_SUPPORT
using HL2UnityPlugin;
#endif

public class ResearchModeVideoStream : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    HL2ResearchMode researchMode;
#endif

    enum DepthSensorMode
    {
        ShortThrow,
        LongThrow,
        None
    };
    [SerializeField] DepthSensorMode depthSensorMode = DepthSensorMode.ShortThrow;
    [SerializeField] bool enablePointCloud = true;
    [SerializeField] GameObject ClippingSphere;
    [SerializeField] GameObject ButtonMenu;
    [SerializeField] GameObject HeadObject;

    TCPClient tcpClient;

    public UnityEngine.UI.Text text;

    public GameObject pointCloudRendererGo;
    public Color pointColor = Color.white;
    public Renderer CaptureStatusLED;
    public GameObject MainCamera;
    private PointCloudRenderer pointCloudRenderer;
    private bool isContinuousSend = false;
    private bool stopPointCloudUpdate = false;
    private float[] pointCloud = new float[] { };
    private float[] clippedPointCloud = new float[] { };
    private byte[] PVTexture = new byte[] { };
    private float refreshInterval = 0.1f;
    private float[] transformationMatrix = new float[] { };


    void Start()
    {

        //Initialize Components for point cloud rendering

        if (pointCloudRendererGo != null)
        {
            pointCloudRenderer = pointCloudRendererGo.GetComponent<PointCloudRenderer>();
        }

        tcpClient = GetComponent<TCPClient>();


#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();

        // By default, Short throw is enabled, note two modes cannot be simutaneously enabled
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.InitializeLongDepthSensor();
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.InitializeDepthSensor();

        //The depth offset will later be calibrated
        researchMode.SetPointCloudDepthOffset(0);

#endif
        StartSensorEvent();
    }

    void LateUpdate()
    {
#if ENABLE_WINMD_SUPPORT
        refreshInterval -= Time.deltaTime;
        transformationMatrix = tcpClient.getTransformationMatrix();
        if (refreshInterval < 0) { //Send at 10fps, change if necessary, note that higher fps might result in bandwidth overflow and the result will become unreadable
            refreshInterval = 0.1f;
            ContinuousSend();
            //Enable this line to activate RGB Data, make sure to import the RGB Plugin instead of Depth only plugin
            //PVTexture = researchMode.GetPVTextureBuffer();
        }
        // Update point cloud
        UpdatePointCloud();
#endif
    }

    bool IsPointIncluded(Vector3 Point)
    {  
        //Determine if a point is inside the Clipping sphere, if true, the point will be used as final points and sent to the computer
        if (ClippingSphere.activeSelf == false)
        {
            return true;
        }
        return Vector3.Distance(ClippingSphere.transform.position, Point) < ClippingSphere.transform.localScale.x ? true : false;
    }

#if ENABLE_WINMD_SUPPORT
    private void UpdatePointCloud()
    {
        if (enablePointCloud && renderPointCloud && pointCloudRendererGo != null) //If point cloud is rendered
        {
            if ((depthSensorMode == DepthSensorMode.LongThrow && !researchMode.LongThrowPointCloudUpdated()) ||
                (depthSensorMode == DepthSensorMode.ShortThrow && !researchMode.PointCloudUpdated())) return;

            //float[] pointCloud = new float[] { };
            if (depthSensorMode == DepthSensorMode.LongThrow) pointCloud = researchMode.GetLongThrowPointCloudBuffer();
            else if (depthSensorMode == DepthSensorMode.ShortThrow) pointCloud = researchMode.GetPointCloudBuffer();
            
            //Getting the point cloud buffer and PV buffer at the same time will crash is 1501 and will result in memory leak in 1432, try with caution

            if (pointCloud.Length > 0)
            {
                int pointCloudLength = pointCloud.Length / 3;
                List<Vector3> pointCloudVector3 = new List<Vector3>();
                for (int i = 0; i < pointCloudLength; i++)
                {
                    Vector3 point = new Vector3(pointCloud[3 * i], pointCloud[3 * i + 1], pointCloud[3 * i + 2]);
                    if (IsPointIncluded(point)) {
                        pointCloudVector3.Add(point);
                    }
                    
                }
                text.text = "Point Cloud Length: " + pointCloudVector3.Count.ToString();
                pointCloudRenderer.Render(pointCloudVector3.ToArray(), pointColor);
                clippedPointCloud = Vector3ToFloatArray(pointCloudVector3.ToArray());
            }

        }
    }
#endif

    private void StartSensorEvent()
    {
#if ENABLE_WINMD_SUPPORT
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.StartLongDepthSensorLoop(enablePointCloud);
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.StartDepthSensorLoop(enablePointCloud);
#endif
    }

    //This is using unity's built in PV Capture. This capture will not crash, but is REALLY slow
    //Feel free to try it and be amazed about how a capture can be this slow in 2022
    //My Nokia N8 is faster than this

    /*private PhotoCapture photoCaptureObject = null;
    private void OnPhotoCaptureCreated(PhotoCapture captureObject)
    {
        stopPointCloudUpdate = true;
        photoCaptureObject = captureObject;

        Resolution cameraResolution = PhotoCapture.SupportedResolutions.OrderByDescending((res) => res.width * res.height).First();

        CameraParameters c = new CameraParameters();
        c.hologramOpacity = 0.0f;
        c.cameraResolutionWidth = cameraResolution.width;
        c.cameraResolutionHeight = cameraResolution.height;
        c.pixelFormat = CapturePixelFormat.BGRA32;

        captureObject.StartPhotoModeAsync(c, OnPhotoModeStarted);
    }

    private void OnPhotoModeStarted(PhotoCapture.PhotoCaptureResult result)
    {
        if (result.success)
        {
            photoCaptureObject.TakePhotoAsync(OnCapturedPhotoToMemory);
        }
        else
        {
            Debug.LogError("Unable to start photo mode!");
        }
    }

    private void OnStoppedPhotoMode(PhotoCapture.PhotoCaptureResult result)
    {
        photoCaptureObject.Dispose();
        photoCaptureObject = null;
        //toggleSensorEvent();
    }

    private void OnCapturedPhotoToMemory(PhotoCapture.PhotoCaptureResult result, PhotoCaptureFrame photoCaptureFrame)
    {
        if (result.success)
        {
            List<byte> imageBufferList = new List<byte>();
            //byte[] imageBufferList = null;
            // Copy the raw IMFMediaBuffer data into our empty byte list.
            photoCaptureFrame.CopyRawImageDataIntoBuffer(imageBufferList);
#if WINDOWS_UWP
            long timestamp = GetCurrentTimestampUnix();
            tcpClient.SendImage(imageBufferList.ToArray(), timestamp);
#endif
        }
        photoCaptureObject.StopPhotoModeAsync(OnStoppedPhotoMode);
        stopPointCloudUpdate = false;
    }*/

#region Button Event Functions
    public void startRGBCapture() //This function is not used since it is just for one time capture debug purpose
    {
        //Built in Unity RGB Capture, if you uncomment the stuff above, you may want to uncomment this
        //PhotoCapture.CreateAsync(false, OnPhotoCaptureCreated);
#if ENABLE_WINMD_SUPPORT
        //This part is Capturing using low level API. However, use with caution since it has a slight chance of crashing the app
        //Note if you uncommented the PVTexture stuff above, you do not need to uncomment this one. That one supports continuous capture
        //However, it do crashes faster in continuous capture
        //var PVTexture = researchMode.GetPVTextureBuffer();
#if WINDOWS_UWP
        long timestamp = GetCurrentTimestampUnix();
        if (tcpClient != null) {
            tcpClient.SendImage(PVTexture, timestamp);
        }
#endif
#endif
    }

    bool renderPointCloud = true;
    public void TogglePointCloudEvent() //If you want to hide point cloud, use this function
    {
        renderPointCloud = !renderPointCloud;
        if (renderPointCloud)
        {
            pointCloudRendererGo.SetActive(true);
        }
        else
        {
            pointCloudRendererGo.SetActive(false);
        }
    }

    public void StopSensorsEvent() //Note: This will stop the sensor AND the app, since the app is the sensor itself :)
    {
#if ENABLE_WINMD_SUPPORT
        researchMode.StopAllSensorDevice();
#endif
    }

    public void SavePointCloudDataEvent()
    {
#if ENABLE_WINMD_SUPPORT
        //This will obtain all the point cloud, which will cram the TCP traffic. Please lower the frame rate if you really want to do this. 40fps will roughly result in a 100Mbps Bandwidth
        //var pointCloud = researchMode.GetPointCloudBuffer();
#if WINDOWS_UWP
        long timestamp = GetCurrentTimestampUnix();
        if (tcpClient != null)
        {
            tcpClient.SendPointCloud(clippedPointCloud, timestamp);
        }
#endif
#endif
    }

    public void BeginContinuousSend()
    {
        isContinuousSend = !isContinuousSend;
        CaptureStatusLED.material.color = isContinuousSend ? Color.green : Color.red;
    }

    public void ContinuousSend()
    {
        if (isContinuousSend)
        {
            SavePointCloudDataEvent();
        }
    }

    public void SendRGBD()
    {
        // For debug use, but can be used by substituting SendRGBD with SavePointCloudDataEvent above
        SavePointCloudDataEvent();
        startRGBCapture();
    }

    public void HideMenu()
    {
        ButtonMenu.SetActive(false);
    }

    public void ShowMenu()
    {
        ButtonMenu.SetActive(true);
    }

    public void Calibrate() //Not Implemented
    {

    }

    public void ToggleClipping()
    {
        ClippingSphere.SetActive(!ClippingSphere.activeSelf);
    }

    public bool RegistrationStarted = false;

    public void BeginRegistration()
    {
        HeadObject.SetActive(!HeadObject.activeSelf); //If the head shows up, the registration starts
    }

    #endregion
    private void OnApplicationFocus(bool focus)
    {
        if (!focus) StopSensorsEvent();
    }

    #region HELPER_FUNCTIONS

    private void ContinuousRegistration()
    {
        if (HeadObject.activeSelf)
        {
            Vector3 regPosition = new Vector3(transformationMatrix[3], transformationMatrix[7], transformationMatrix[11]);
            Matrix4x4 transformationMatrix4x4 = new Matrix4x4();
            for (int i = 0; i < 16; i++)
            {
                transformationMatrix4x4[i] = transformationMatrix[i];
            }
            if (transformationMatrix4x4.ValidTRS()) //We will make sure the transformation matrix is valid before proceeding, just in case the data is corrupt
            {
                HeadObject.transform.position = regPosition;
                HeadObject.transform.rotation = transformationMatrix4x4.rotation;
            }
        }
    }

    private float[] Vector3ToFloatArray(Vector3[] points)
    {
        float[] posxyz = new float[points.Length*3];
        for(int i = 0; i < points.Length; i++)
        {
            posxyz[3 * i] = points[i].x;
            posxyz[3 * i + 1] = points[i].y;
            posxyz[3 * i + 2] = points[i].z;
        }
        return posxyz;
    }

#if WINDOWS_UWP
    private long GetCurrentTimestampUnix()
    {
        // Get the current time, in order to create a PerceptionTimestamp. 
        Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
        Windows.Perception.PerceptionTimestamp ts = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
        return ts.TargetTime.ToUnixTimeMilliseconds();
        //return ts.SystemRelativeTargetTime.Ticks;
    }
    private Windows.Perception.PerceptionTimestamp GetCurrentTimestamp()
    {
        // Get the current time, in order to create a PerceptionTimestamp. 
        Windows.Globalization.Calendar c = new Windows.Globalization.Calendar();
        return Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(c.GetDateTime());
    }
#endif
    #endregion
}