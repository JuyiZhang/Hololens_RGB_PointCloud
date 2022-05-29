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

    TCPClient tcpClient;

    public UnityEngine.UI.Text text;

    public GameObject pointCloudRendererGo;
    public Color pointColor = Color.white;
    public Renderer CaptureStatusLED;
    public GameObject MainCamera;
    private PointCloudRenderer pointCloudRenderer;
    private bool isContinuousSend = false;
    private bool stopPointCloudUpdate = false;
    private bool sensorStopped = true;
    private float[] pointCloud = new float[] { };
    private float[] transformationMatrix = new float[] { };
    private byte[] PVTexture = new byte[] { };
    private float refreshInterval = 0.5f;

#if ENABLE_WINMD_SUPPORT
    Windows.Perception.Spatial.SpatialCoordinateSystem unityWorldOrigin;
#endif

    private void Awake()
    {
#if ENABLE_WINMD_SUPPORT
#if UNITY_2020_1_OR_NEWER // note: Unity 2021.2 and later not supported
        //IntPtr WorldOriginPtr = Microsoft.MixedReality.Toolkit.WindowsMixedReality.WindowsMixedRealityUtilities.UtilitiesProvider.ISpatialCoordinateSystemPtr;
        IntPtr WorldOriginPtr = UnityEngine.XR.WindowsMR.WindowsMREnvironment.OriginSpatialCoordinateSystem;
        unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
        //unityWorldOrigin = Windows.Perception.Spatial.SpatialLocator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem;
#else
        IntPtr WorldOriginPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
        unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#endif
#endif
    }
    void Start()
    {

        if (pointCloudRendererGo != null)
        {
            pointCloudRenderer = pointCloudRendererGo.GetComponent<PointCloudRenderer>();
        }

        tcpClient = GetComponent<TCPClient>();


#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.InitializeLongDepthSensor();
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.InitializeDepthSensor();
        
        researchMode.SetReferenceCoordinateSystem(unityWorldOrigin);
        researchMode.SetPointCloudDepthOffset(0);

#endif
        StartSensorEvent();
        tcpClient.ConnectToServerEvent();
        isContinuousSend = true;
        //sensorStopped = false;
    }

    void LateUpdate()
    {
        refreshInterval -= Time.deltaTime;
        if (refreshInterval < 0)
        {
            refreshInterval = 0.1f; //Change the value here to make refresh time of transformation matrix longer or shorter
            transformationMatrix = tcpClient.getTransformationMatrix();
            text.text = transformationMatrix[0].ToString() + "," + transformationMatrix[1].ToString() + "," + transformationMatrix[2].ToString() + "," + transformationMatrix[3].ToString();
            ContinuousSend();
        }

#if ENABLE_WINMD_SUPPORT
        // Update point cloud
        UpdatePointCloud();
        
#endif
        //text.text = MainCamera.transform.position.ToString();
    }

#if ENABLE_WINMD_SUPPORT
    private void UpdatePointCloud()
    {
        if (enablePointCloud && renderPointCloud && pointCloudRendererGo != null)
        {
            if ((depthSensorMode == DepthSensorMode.LongThrow && !researchMode.LongThrowPointCloudUpdated()) ||
                (depthSensorMode == DepthSensorMode.ShortThrow && !researchMode.PointCloudUpdated())) return;

            //float[] pointCloud = new float[] { };
            if (depthSensorMode == DepthSensorMode.LongThrow) pointCloud = researchMode.GetLongThrowPointCloudBuffer();
            else if (depthSensorMode == DepthSensorMode.ShortThrow) pointCloud = researchMode.GetPointCloudBuffer();
            
            if (pointCloud.Length > 0)
            {
                int pointCloudLength = pointCloud.Length / 3;
                Vector3[] pointCloudVector3 = new Vector3[pointCloudLength];
                for (int i = 0; i < pointCloudLength; i++)
                {
                    pointCloudVector3[i] = new Vector3(pointCloud[3 * i], pointCloud[3 * i + 1], pointCloud[3 * i + 2]);
                }
                //text.text = "Point Cloud Length: " + pointCloudVector3.Length.ToString();
                pointCloudRenderer.Render(pointCloudVector3, pointColor);
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
        sensorStopped = false;
    }

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
    public void startRGBCapture()
    {
        //toggleSensorEvent();
        //PhotoCapture.CreateAsync(false, OnPhotoCaptureCreated);
#if ENABLE_WINMD_SUPPORT
        //var PVImage = researchMode.GetPVTextureBuffer();
#if WINDOWS_UWP
        long timestamp = GetCurrentTimestampUnix();
        if (tcpClient != null) {
            tcpClient.SendImage(PVTexture, timestamp);
        }
#endif
#endif
    }

    public void startRGBDCapture()
    {
        SendRGBD();
    }

    bool renderPointCloud = true;
    public void TogglePointCloudEvent()
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

    public void StopSensorsEvent()
    {
#if ENABLE_WINMD_SUPPORT
        researchMode.StopAllSensorDevice();
#endif
        sensorStopped = true;
    }

    public void SavePointCloudDataEvent()
    {
#if ENABLE_WINMD_SUPPORT
        //var pointCloud = researchMode.GetPointCloudBuffer();
#if WINDOWS_UWP
        long timestamp = GetCurrentTimestampUnix();
        if (tcpClient != null)
        {
            tcpClient.SendPointCloud(pointCloud, timestamp);
        }
#endif
#endif
    }

    /*public void toggleSensorEvent()
    {
#if ENABLE_WINMD_SUPPORT
        if (sensorStopped) {
            researchMode.SetDepthSensorLoopState(true);
            text.text = "ResearchModeSensorStarted";
        } else {
            researchMode.SetDepthSensorLoopState(false);
            text.text = "ResearchModeSensorStopped";
        }
        sensorStopped = !sensorStopped;
#endif
    }*/

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
        SavePointCloudDataEvent();
        //yield return new WaitForSeconds(1f);
        startRGBCapture();
        //yield return new WaitForSeconds(1f);
    }

    #endregion
    private void OnApplicationFocus(bool focus)
    {
        if (!focus) StopSensorsEvent();
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
}