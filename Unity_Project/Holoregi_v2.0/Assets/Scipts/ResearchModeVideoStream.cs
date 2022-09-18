using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

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
    [SerializeField] GameObject SourceContainer;
    private float[] transformationMatrix = new float[] { };

    TCPClient tcpClient;

    public GameObject depthPreviewPlane = null;
    private Material depthMediaMaterial = null;
    private Texture2D depthMediaTexture = null;
    private byte[] depthFrameData = null;
    public GameObject phantomHead;
    public GameObject patientMRI;
    public GameObject patientCT;

    public GameObject shortAbImagePreviewPlane = null;
    private Material shortAbImageMediaMaterial = null;
    private Texture2D shortAbImageMediaTexture = null;
    private byte[] shortAbImageFrameData = null;

    public GameObject longDepthPreviewPlane = null;
    private Material longDepthMediaMaterial = null;
    private Texture2D longDepthMediaTexture = null;
    private byte[] longDepthFrameData = null;

    public GameObject longAbImagePreviewPlane = null;
    private Material longAbImageMediaMaterial = null;
    private Texture2D longAbImageMediaTexture = null;
    private byte[] longAbImageFrameData = null;

    public GameObject LFPreviewPlane = null;
    private Material LFMediaMaterial = null;
    private Texture2D LFMediaTexture = null;
    private byte[] LFFrameData = null;

    public GameObject RFPreviewPlane = null;
    private Material RFMediaMaterial = null;
    private Texture2D RFMediaTexture = null;
    private byte[] RFFrameData = null;
    private int numSendFrames = 0;

    public UnityEngine.UI.Text text;
    public UnityEngine.UI.Text serverFeedbackText;

    public GameObject pointCloudRendererGo;
    public Color pointColor = Color.white;
    private PointCloudRenderer pointCloudRenderer;
    private float[] clippedPointCloud = new float[] { };
#if ENABLE_WINMD_SUPPORT
    Windows.Perception.Spatial.SpatialCoordinateSystem unityWorldOrigin;
#endif

    private bool IsPointIncluded(Vector3 Point)
    {
        //Determine if a point is inside the Clipping sphere, if true, the point will be used as final points and sent to the computer
        if (ClippingSphere.activeSelf == false)
        {
            return true;
        }
        return Vector3.Distance(ClippingSphere.transform.position, Point) < ClippingSphere.transform.localScale.z ? true : false;
    }
    public void ToggleClipping()
    {
        ClippingSphere.SetActive(!ClippingSphere.activeSelf);
    }

    private void Awake()
    {
#if ENABLE_WINMD_SUPPORT
#if UNITY_2020_1_OR_NEWER // note: Unity 2021.2 and later not supported
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
     
        if (depthSensorMode == DepthSensorMode.ShortThrow)
        {
            if (depthPreviewPlane != null)
            {
                depthMediaMaterial = depthPreviewPlane.GetComponent<MeshRenderer>().material;
                depthMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
                depthMediaMaterial.mainTexture = depthMediaTexture;
            }

            if (shortAbImagePreviewPlane != null)
            {
                shortAbImageMediaMaterial = shortAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
                shortAbImageMediaTexture = new Texture2D(512, 512, TextureFormat.Alpha8, false);
                shortAbImageMediaMaterial.mainTexture = shortAbImageMediaTexture;
            }
            longDepthPreviewPlane.SetActive(false);
            longAbImagePreviewPlane.SetActive(false);
        }
        
        if (depthSensorMode == DepthSensorMode.LongThrow)
        {
            if (longDepthPreviewPlane != null)
            {
                longDepthMediaMaterial = longDepthPreviewPlane.GetComponent<MeshRenderer>().material;
                longDepthMediaTexture = new Texture2D(320, 288, TextureFormat.Alpha8, false);
                longDepthMediaMaterial.mainTexture = longDepthMediaTexture;
            }

            if (longAbImagePreviewPlane != null)
            {
                longAbImageMediaMaterial = longAbImagePreviewPlane.GetComponent<MeshRenderer>().material;
                longAbImageMediaTexture = new Texture2D(320, 288, TextureFormat.Alpha8, false);
                longAbImageMediaMaterial.mainTexture = longAbImageMediaTexture;
            }
            depthPreviewPlane.SetActive(false);
            shortAbImagePreviewPlane.SetActive(false);
        }
        

        if (LFPreviewPlane != null)
        {
            LFMediaMaterial = LFPreviewPlane.GetComponent<MeshRenderer>().material;
            LFMediaTexture = new Texture2D(640, 480, TextureFormat.Alpha8, false);
            LFMediaMaterial.mainTexture = LFMediaTexture;
        }

        if (RFPreviewPlane != null)
        {
            RFMediaMaterial = RFPreviewPlane.GetComponent<MeshRenderer>().material;
            RFMediaTexture = new Texture2D(640, 480, TextureFormat.Alpha8, false);
            RFMediaMaterial.mainTexture = RFMediaTexture;
        }

        if (pointCloudRendererGo != null)
        {
            pointCloudRenderer = pointCloudRendererGo.GetComponent<PointCloudRenderer>();
        }

        tcpClient = GetComponent<TCPClient>();
        tcpClient.ConnectToServerEvent();


        // for debugging 

        //transformationMatrix = new float[] {
        //1,0,0,1,
        //0,1,0,0,
        //0,0,1,0,
        //0,0,0,1,};
        //transformationMatrix = new float[] {
        //0.5000000f, -0.5000000f, 0.7071068f,0f,
        //0.8535534f, 0.1464466f, -0.5000000f,0,
        //0.1464466f, 0.8535534f, 0.5000000f,0,
        //0,0,0,1};
        //UpdateHeadTransformation();


#if ENABLE_WINMD_SUPPORT
        researchMode = new HL2ResearchMode();

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.InitializeLongDepthSensor();
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.InitializeDepthSensor();
        
        researchMode.InitializeSpatialCamerasFront();
        researchMode.SetReferenceCoordinateSystem(unityWorldOrigin);
        
        
        researchMode.SetPointCloudDepthOffset(0);

        // Depth sensor should be initialized in only one mode
        if (depthSensorMode == DepthSensorMode.LongThrow) researchMode.StartLongDepthSensorLoop(enablePointCloud);
        else if (depthSensorMode == DepthSensorMode.ShortThrow) researchMode.StartDepthSensorLoop(enablePointCloud);

        researchMode.StartSpatialCamerasFrontLoop();
#endif
        TestPhantomHead();
    }

    bool startRealtimePreview = true;
    void LateUpdate()
    {

#if ENABLE_WINMD_SUPPORT
        serverFeedbackText.text=tcpClient.getServerFeedback();
        //SourceContainer.transform.localScale=new Vector3(1f,1f,1f);
        //serverFeedbackText.text=SourceContainer.transform.localToWorldMatrix.ToString();
        //SourceContainer.transform.localScale=new Vector3(0.001f,0.001f,0.001f);
        if (tcpClient.updatedTransformation)
        {

            transformationMatrix = tcpClient.getTransformationMatrix();
            UpdateHeadTransformation();
            tcpClient.updatedTransformation=false;
        }



        // Update point cloud
        UpdatePointCloud();
#endif
    }

#if ENABLE_WINMD_SUPPORT
    private void UpdatePointCloud()
    {
        if (enablePointCloud && renderPointCloud && pointCloudRendererGo != null)
        {
            if ((depthSensorMode == DepthSensorMode.LongThrow && !researchMode.LongThrowPointCloudUpdated()) ||
                (depthSensorMode == DepthSensorMode.ShortThrow && !researchMode.PointCloudUpdated())) return;

            float[] pointCloud = new float[] { };
            if (depthSensorMode == DepthSensorMode.LongThrow) pointCloud = researchMode.GetLongThrowPointCloudBuffer();
            else if (depthSensorMode == DepthSensorMode.ShortThrow) pointCloud = researchMode.GetPointCloudBuffer();
            
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

    public void sendCurrentFrame()
    {
#if WINDOWS_UWP
        numSendFrames+=1;
        long timestamp = GetCurrentTimestampUnix();
        if (tcpClient != null)
        {
            tcpClient.SendPointCloud(clippedPointCloud, timestamp);
        }
#endif
    }
    private void UpdateHeadTransformation()
    {
        //M_r=[r_x,u_x,l_x,p_x,
        //     r_y,u_y,l_y,p_y,
        //     r_z,u_z,l_z,p_z,
        //     0,0,0,1]

        //M_l=[r_x,-u_x,-l_x,-p_x,
        //     -r_y,u_y,l_y,p_y,
        //     -r_z,u_z,l_z,p_z,
        //     0,0,0,1]
        
        // it's important that the transformation matrix in python is row-major
        // while it's column-major in unity!!!!!!!!!!!!!!!!


        float[] transformationMatrixInUnity = new float[] { transformationMatrix[0], -transformationMatrix[4], -transformationMatrix[8], -transformationMatrix[12],
                                                            -transformationMatrix[1], transformationMatrix[5], transformationMatrix[9], transformationMatrix[13],
                                                            -transformationMatrix[2], transformationMatrix[6], transformationMatrix[10], transformationMatrix[14],
                                                            -transformationMatrix[3], transformationMatrix[7], transformationMatrix[11], transformationMatrix[15],
                                                            };


        Vector3 regPosition = new Vector3(-transformationMatrix[3], transformationMatrix[7], transformationMatrix[11]);
        Matrix4x4 transformationMatrix4x4 = new Matrix4x4();
        for (int i = 0; i < 16; i++)
        {
            transformationMatrix4x4[i] = transformationMatrixInUnity[i];
        }
        SourceContainer.transform.position = regPosition;
        SourceContainer.transform.rotation = transformationMatrix4x4.rotation;
    }
    public void RequestData()
    {
#if WINDOWS_UWP
        long timestamp = GetCurrentTimestampUnix();
        if (tcpClient != null)
        {   SourceContainer.transform.localScale=new Vector3(1f,1f,1f);
            
    
            Matrix4x4 transformationMatrix4x4 =  SourceContainer.transform.localToWorldMatrix;
            SourceContainer.transform.localScale=new Vector3(0.001f,0.001f,0.001f);
            float[] transformationMatrix=new float[16];
            for (int i=0;i<16;i++)
            {
                transformationMatrix[i]=transformationMatrix4x4[i];
            }
            tcpClient.RequestData(transformationMatrix, timestamp);
        };
        
#endif
    }

    private float[] Vector3ToFloatArray(Vector3[] points)
    {
        float[] posxyz = new float[points.Length * 3];
        for (int i = 0; i < points.Length; i++)
        {
            posxyz[3 * i] = points[i].x;
            posxyz[3 * i + 1] = points[i].y;
            posxyz[3 * i + 2] = points[i].z;
        }
        return posxyz;
    }
    public void TestPhantomHead()
    {
        phantomHead.SetActive(true);
        patientCT.SetActive(false);
        patientMRI.SetActive(false);
        tcpClient.SendMessage("tPhantomHead");
        ResetRegistration();
    }
    public void TestCT()
    {
        phantomHead.SetActive(false);
        patientCT.SetActive(true);
        patientMRI.SetActive(false);
        tcpClient.SendMessage("tPatientCT");
        ResetRegistration();
    }
    public void TestMRI()
    {
        phantomHead.SetActive(false);
        patientCT.SetActive(false);
        patientMRI.SetActive(true);
        tcpClient.SendMessage("tPatientMRI");
        ResetRegistration();
    }
    private void ResetRegistration()
    {
        SourceContainer.transform.position=new Vector3(0,0,0);
        SourceContainer.transform.localRotation=Quaternion.identity;
    }

    #region Button Event Functions
    public void TogglePreviewEvent()
    {
        startRealtimePreview = !startRealtimePreview;
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
        startRealtimePreview = false;
    }

    public void SaveAHATSensorDataEvent()
    {
#if ENABLE_WINMD_SUPPORT
        var depthMap = researchMode.GetDepthMapBuffer();
        var AbImage = researchMode.GetShortAbImageBuffer();
#if WINDOWS_UWP
        if (tcpClient != null)
        {
            tcpClient.SendUINT16Async(depthMap, AbImage);
        }
#endif
#endif
    }

    public void SaveSpatialImageEvent()
    {
#if ENABLE_WINMD_SUPPORT
#if WINDOWS_UWP
        long ts_ft_left, ts_ft_right;
        var LRFImage = researchMode.GetLRFCameraBuffer(out ts_ft_left, out ts_ft_right);
        Windows.Perception.PerceptionTimestamp ts_left = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(DateTime.FromFileTime(ts_ft_left));
        Windows.Perception.PerceptionTimestamp ts_right = Windows.Perception.PerceptionTimestampHelper.FromHistoricalTargetTime(DateTime.FromFileTime(ts_ft_right));

        long ts_unix_left = ts_left.TargetTime.ToUnixTimeMilliseconds();
        long ts_unix_right = ts_right.TargetTime.ToUnixTimeMilliseconds();
        long ts_unix_current = GetCurrentTimestampUnix();

        text.text = "Left: " + ts_unix_left.ToString() + "\n" +
            "Right: " + ts_unix_right.ToString() + "\n" +
            "Current: " + ts_unix_current.ToString();

        if (tcpClient != null)
        {
            //tcpClient.SendSpatialImageAsync(LRFImage, ts_unix_left, ts_unix_right);
        }
#endif
#endif
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