from logging import exception
import socket
import struct
import sys
import os
import numpy as np
import cv2
import time
#import open3d as o3d
import pickle as pkl

def tcp_server():
    serverHost = '192.168.1.100' # localhost, find the ip of your *computer*
    serverPort = 9090
    save_folder = 'data/'

    if not os.path.isdir(save_folder):
        os.mkdir(save_folder)

    # Create a socket
    sSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind server to port
    try:
        sSock.bind((serverHost, serverPort))
        print('Server bind to port '+str(serverPort))
    except socket.error as msg:
        print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
        return

    sSock.listen(10)
    print('Start listening...')
    sSock.settimeout(3.0)
    while True:
        try:
            conn, addr = sSock.accept() # Blocking, wait for incoming connection
            break
        except KeyboardInterrupt:
            sys.exit(0)
        except Exception:
            continue

    print('Connected with ' + addr[0] + ':' + str(addr[1]))

    while True:
        # Receiving from client
        try:
            data = conn.recv(1024*1024*4+100)
            if len(data)==0:
                continue
            header = data[0:1].decode('utf-8')
            print('--------------------------\nHeader: ' + header)
            print(len(data))

            if header == 's':
                # save depth sensor images
                data_length = struct.unpack(">i", data[1:5])[0]
                N = data_length
                depth_img_np = np.frombuffer(data[5:5+N], np.uint16).reshape((512,512))
                ab_img_np = np.frombuffer(data[5+N:5+2*N], np.uint16).reshape((512,512))
                timestamp = str(int(time.time()))
                cv2.imwrite(save_folder + timestamp+'_depth.tiff', depth_img_np)
                cv2.imwrite(save_folder + timestamp+'_abImage.tiff', ab_img_np)
                print('Image with ts ' + timestamp + ' is saved')
            if header == 'f':
                # save spatial camera images
                data_length = struct.unpack(">i", data[1:5])[0]
                ts_left, ts_right = struct.unpack(">qq", data[5:21])

                N = int(data_length/2)
                LF_img_np = np.frombuffer(data[21:21+N], np.uint8).reshape((480,640))
                RF_img_np = np.frombuffer(data[21+N:21+2*N], np.uint8).reshape((480,640))
                cv2.imwrite(save_folder + str(ts_left)+'_LF.tiff', LF_img_np)
                cv2.imwrite(save_folder + str(ts_right)+'_RF.tiff', RF_img_np)
                print('Image with ts %d and %d is saved' % (ts_left, ts_right))
            if header == 'p':
                # save point cloud
                # print(data)
                timestamp = struct.unpack(">q", data[1:9])[0]
                N_pointcloud = struct.unpack(">q", data[9:17])[0]
                print("Length of point cloud:" + str(N_pointcloud))
                pointcloud_np = np.frombuffer(data[17:17+N_pointcloud*4], np.float32).reshape((-1,3))
                timestamp = str(int(time.time()*1000))
                filename = "PointCloudCapture/" + timestamp + "_pc.ndarray"
                print(filename)
                np.save(filename, pointcloud_np, allow_pickle=True, fix_imports=True)
                #transformationMatrix = np.float32([2.5,1.5,1.0,1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0])
                #conn.send(transformationMatrix.tobytes())
                #Reconstruction data, remove comment in the future
                """temp_filename_pc = timestamp + '_pc.ply'
                #print(pointcloud_np.shape)
                #o3d_pc = o3d.geometry.PointCloud()
                #o3d_pc.points = o3d.utility.Vector3dVector(pointcloud_np.astype(np.float64))
                #o3d.io.write_point_cloud(save_folder + temp_filename_pc, o3d_pc, write_ascii=True)
                #print('Saved  image to ' + temp_filename_pc)"""
            if header == "i":
                # save RGB Image
                print("Saving RGB Image")
                timestamp = struct.unpack(">q", data[1:9])[0]
                N_image = struct.unpack(">q", data[9:17])[0]
                print("Length of point cloud:" + str(N_image))
                #image_np = np.frombuffer(data[17:17+N_image*4], np.uint8).reshape((-1,3))
                timestamp = str(int(time.time()*1000))
                filename = "ImageCapture/" + timestamp + "_pc.imgdata"
                f = open(filename, 'wb')
                f.write(data[17:17+N_image*4])
                f.close()
                #print(filename)
                #np.save(filename, image_np, allow_pickle=True, fix_imports=True)

        except Exception as excep:
            print(type(excep))
            print(excep.args)
            #break
    
    print('Closing socket...')
    sSock.close()


if __name__ == "__main__":
    tcp_server()
