using System;
using System.Threading;
using UnityEngine;
//using System.Runtime.Serialization.Formatters.Binary;
#if WINDOWS_UWP
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

public class TCPClient : MonoBehaviour
{
    #region Unity Functions

    private void Awake()
    {
        ConnectionStatusLED.material.color = Color.red;
    }
    private void OnApplicationFocus(bool focus)
    {
        if (!focus)
        {
#if WINDOWS_UWP
            StopConnection();
#endif
        }
    }
    #endregion // Unity Functions

    [SerializeField]
    string hostIPAddress, port;

    public Renderer ConnectionStatusLED;
    public float[] transformationMatrix = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}; //Default identity matrix
    private bool connected = false;
    private Thread clientRcvThread;
    public bool Connected
    {
        get { return connected; }
    }

    //This part are from Peter Gu's Hololens 2 Research Mode for Unity Repository directly, refer to the directory for more info
#if WINDOWS_UWP
    StreamSocket socket = null;
    public DataWriter dw;
    public DataReader dr;
    private async void StartConnection()
    {
        if (socket != null) socket.Dispose();

        try
        {
            socket = new StreamSocket();
            var hostName = new Windows.Networking.HostName(hostIPAddress);
            await socket.ConnectAsync(hostName, port);
            dw = new DataWriter(socket.OutputStream);
            dr = new DataReader(socket.InputStream);
            dr.InputStreamOptions = InputStreamOptions.Partial;
            connected = true;
            ConnectionStatusLED.material.color = Color.green;

            // This is the original part where I created the receive thread for transformation matrix
            clientRcvThread = new Thread (new ThreadStart(ListenForData));
            clientRcvThread.IsBackground = true;
            clientRcvThread.Start();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
    }

    private void StopConnection()
    {
        dw?.DetachStream();
        dw?.Dispose();
        dw = null;

        dr?.DetachStream();
        dr?.Dispose();
        dr = null;

        socket?.Dispose();
        connected = false;
        ConnectionStatusLED.material.color = Color.red;
    }


    // This is the thread where we listen for transformation matrix and updates it
    private async void ListenForData() {
        Byte[] bytes = new Byte[sizeof(float)*16];
        while (true) {
            await dr.LoadAsync(sizeof(float)*16);
            dr.ReadBytes(bytes);
            transformationMatrix = BytesToFloat(bytes);
        }
    }


    // The following part comes from peter gu's repository but it is not really used
    bool lastMessageSent = true;
    public async void SendUINT16Async(ushort[] data)
    {
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try
        {
            // Write header
            dw.WriteString("s"); // header "s" 

            // Write point cloud
            dw.WriteInt32(data.Length);
            dw.WriteBytes(UINT16ToBytes(data));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;
    }

    public async void SendUINT16Async(ushort[] data1, ushort[] data2)
    {
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try
        {
            // Write header
            dw.WriteString("s"); // header "s" stands for it is ushort array (uint16)

            // Write Length
            dw.WriteInt32(data1.Length + data2.Length);

            // Write actual data
            dw.WriteBytes(UINT16ToBytes(data1));
            dw.WriteBytes(UINT16ToBytes(data2));

            // Send out
            await dw.StoreAsync();
            await dw.FlushAsync();
        }
        catch (Exception ex)
        {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;
    }

    // This is the part I wrote to send the point cloud
    // The timestamp will be used to match the point cloud data with the rgb data
    public async void SendPointCloud(float[] pointCloud, long timestamp) {
    
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try {
            dw.WriteString("p"); //header "p" for point cloud
            dw.WriteInt64(timestamp);
            dw.WriteInt64(pointCloud.Length); //length of float elements

            dw.WriteBytes(FloatToBytes(pointCloud)); //write actual data

            await dw.StoreAsync();
            await dw.FlushAsync();
        } catch (Exception ex) {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;

    }

    //This is the part I wrote to send the RGB Image
    public async void SendImage(byte[] RGBImage, long timestamp) {
    
        if (!lastMessageSent) return;
        lastMessageSent = false;
        try {
            dw.WriteString("i"); //header "i" for image
            dw.WriteInt64(timestamp);
            dw.WriteInt64(RGBImage.Length); //length of float elements

            dw.WriteBytes(RGBImage); //write actual data

            await dw.StoreAsync();
            await dw.FlushAsync();
        } catch (Exception ex) {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;

    }

    //Not really used, just out of curiousity
    public async void SendPointCloudV3(Vector3[] arrayVertices) {
    
        if (!lastMessageSent) return;
        lastMessageSent = false;
        var floatElement = arrayVertices.Length * 3;
        float[] vertexConvertedData = new float[floatElement];
        for(int i=0; i<arrayVertices.Length; i++) {
            vertexConvertedData[i*3] = arrayVertices[i].x;
            vertexConvertedData[i*3+1] = arrayVertices[i].y;
            vertexConvertedData[i*3+2] = arrayVertices[i].z;
        }
        try {
            dw.WriteString("p"); //header "p" for point cloud
            dw.WriteInt64(floatElement); //length of float elements
            dw.WriteBytes(FloatToBytes(vertexConvertedData)); //write actual data
            await dw.StoreAsync();
            //await dw.flushAsync();
        } catch (Exception ex) {
            SocketErrorStatus webErrorStatus = SocketError.GetStatus(ex.GetBaseException().HResult);
            Debug.Log(webErrorStatus.ToString() != "Unknown" ? webErrorStatus.ToString() : ex.Message);
        }
        lastMessageSent = true;

    }

#endif


    #region Helper Function
    byte[] UINT16ToBytes(ushort[] data)
    {
        byte[] ushortInBytes = new byte[data.Length * sizeof(ushort)];
        System.Buffer.BlockCopy(data, 0, ushortInBytes, 0, ushortInBytes.Length);
        return ushortInBytes;
    }

    byte[] FloatToBytes(float[] data)
    {
        byte[] floatInBytes = new byte[data.Length * sizeof(float)];
        System.Buffer.BlockCopy(data, 0, floatInBytes, 0, floatInBytes.Length);
        return floatInBytes;
    }

    float[] BytesToFloat(byte[] data)
    {
        float[] bytesInFloat = new float[data.Length/sizeof(float)];
        System.Buffer.BlockCopy(data, 0, bytesInFloat, 0, data.Length);
        return bytesInFloat;
    }
    #endregion

    #region Button Callback
    public void ConnectToServerEvent()
    {
        Debug.Log("Connection altered");
#if WINDOWS_UWP
        if (!connected) StartConnection();
        else StopConnection();
#endif
    }
    #endregion

    #region Properties
    public float[] getTransformationMatrix()
    {
        return transformationMatrix;
    }
    #endregion
}
