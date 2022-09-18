from logging import exception
import socket
import struct
import sys
import os
import numpy as np
import cv2
import time
import open3d as o3d
import pickle as pkl
from preprocess import reconstruct_pcd, downsample_denoise, mesh_to_pcd_downsample_mri
from registration import registration
import shutil
import copy
import datetime

sourceType="PhantomHead"
# sourceType="PatientMRI"
entrance=1

def initCaptureFolder(saveFolder):
    now=datetime.datetime.now().strftime("_%m_%d_%Y_%H_%M_%S")
    if not os.path.isdir(saveFolder):
        os.mkdir(saveFolder)
    else:
        os.rename(saveFolder,saveFolder+f"_{sourceType}_"+now)
        os.mkdir(saveFolder)
        shutil.rmtree(saveFolder)
        os.mkdir(saveFolder)


def tcp_server():
    global sourceType
    # serverHost = '10.5.87.195' # localhost, find the ip of your *computer*
    # serverHost = '192.168.1.4' #home
    serverHost = '192.168.1.10' # gf
    serverPort = 9090
    saveFolder = './data/PointCloudCapture'
    initCaptureFolder(saveFolder)


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

            if header == 'p':
                # save point cloud
                timestamp = struct.unpack(">q", data[1:9])[0]
                N_pointcloud = struct.unpack(">q", data[9:17])[0]
                print("Length of point cloud:" + str(N_pointcloud))
                pointcloud_np = np.frombuffer(data[17:17+N_pointcloud*4], np.float32).reshape((-1,3))
                timestamp = str(int(time.time()*1000))
                filename = "./data/PointCloudCapture/" + timestamp + "_pc.ndarray"
                print(filename)
                np.save(filename, pointcloud_np, allow_pickle=True, fix_imports=True)

            if header=="t":
                sourceType=data[1:].decode('utf-8')
                initCaptureFolder(saveFolder)
                print(f"current source type: {sourceType}")
                
            if header == 'r':
                pcd_coord = getCoordinate()

                print('receive an request')
                T_Unity = np.frombuffer(data[1:(1+4*16)], np.float32).reshape((4, 4))
                T_O3D=convertTransformationToOpen3d(T_Unity)
                print(f"T of MRI:\n {T_O3D}")
                # assume the mri mesh is in this folder with this name
                # sourcePath = "./realPatient.stl"
                print(f"source type: {sourceType}")
                if sourceType == "PhantomHead":
                    sourcePath = "./phantomHeadCrop.stl"
                elif sourceType == "PatientCT":
                    sourcePath = "./patientCTCrop.stl"
                else:
                    sourcePath = "./patientMRICrop.stl"

                # stitch all pieces
                scan = reconstruct_pcd()
                # downsample/denoise
                scan = downsample_denoise(scan)
                # o3d.visualization.draw_geometries([scan,pcd_coord])
                # mri mesh to pcd (downsample)
                mri = mesh_to_pcd_downsample_mri(sourcePath)
                # T_O3D=np.asarray([
                #     -1,0,0,0,
                #     0,1,0,0,
                #     0,0,1,0,
                #     0,0,0,1
                # ]).reshape(4,4)
                # mri=mri.transform(T_O3D)
                # o3d.visualization.draw_geometries([mri,pcd_coord,scan])
                # # registration
                transformationMatrix = registration(mri, scan)
                # draw_registration_result(mri, scan, transformationMatrix)


                print(transformationMatrix)
                # draw_registration_result(mri, scan, transformationMatrix)
                conn.send(transformationMatrix.reshape(-1).astype(np.float32).tobytes())
                # print(f"sent data:\n {transformationMatrix}")
                draw_registration_result(mri, scan, transformationMatrix)

                
            # if header == "i":
            #     # save RGB Image
            #     print("Saving RGB Image")
            #     timestamp = struct.unpack(">q", data[1:9])[0]
            #     N_image = struct.unpack(">q", data[9:17])[0]
            #     print("Length of point cloud:" + str(N_image))
            #     #image_np = np.frombuffer(data[17:17+N_image*4], np.uint8).reshape((-1,3))
            #     timestamp = str(int(time.time()*1000))
            #     filename = "ImageCapture/" + timestamp + "_pc.imgdata"
            #     f = open(filename, 'wb')
            #     f.write(data[17:17+N_image*4])
            #     f.close()
            #     #print(filename)
            #     #np.save(filename, image_np, allow_pickle=True, fix_imports=True)

        except Exception as excep:
            print(type(excep))
            print(excep.args)
            #break
    
    print('Closing socket...')
    sSock.close()

def convertTransformationToOpen3d(matrixInUnity):
    matrixInO3D=np.copy(matrixInUnity)
    matrixInO3D=matrixInO3D.T
    matrixInO3D[0,:]=-matrixInO3D[0,:]
    return matrixInO3D


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def getCoordinate():
    xyz = np.asarray([
        0, 0, 0,
        1, 0, 0,
        2, 0, 0,
        3, 0, 0,
        0, 1, 0,
        0, 2, 0,
        0, 3, 0,
        0, 4, 0,
        0, 5, 0,
        0, 6, 0,
        0, 0, 1,
        0, 0, 2,
        0, 0, 3,
        0, 0, 4,
        0, 0, 5,
        0, 0, 6,
        0, 0, 7,
        0, 0, 8,
        0, 0, 9,
        0, 0, 10,
        0, 0, 11,
        0, 0, 12,
        0, 0, 13,
        0, 0, -1,
        0, 0, -2,
        0, 0, -3,
        0, 0, -4,
        0, 0, -5,
        0, 0, -6,
        0, 0, -7,
        0, 0, -8,
        0, 0, -9,
        0, 0, -10,
        0, 0, -11,
        0, 0, -12,
        0, 0, -13,
        0, 0, -14,
        0, 0, -15,
        0, 0, -16,
        0, 0, -17,
        0, 0, -18,
        0, 0, -19,
        0, 0, -20,
        0, 0, -21,
        0, 0, -22,
        0, 0, -23,
        0, 0, -24,
        0, 0, -25,

    ]).reshape(-1, 3) * 0.01

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd

if __name__ == "__main__":
    if entrance==1:
        tcp_server()
    elif entrance==2:
        # sourceType="PatientMRI"

        pcd_coord=getCoordinate()

        # o3d.visualization.draw_geometries([pcd_coord])
        # path = "./realPatient.stl"
        if sourceType == "PhantomHead":
            sourcePath = "./phantomHeadCrop.stl"
        elif sourceType == "PatientCT":
            # sourcePath = "./patientCT.stl"
            sourcePath = "./patientCTCrop.stl"
        else:
            # sourcePath = "./patientMRI.stl"
            sourcePath = "./patientMRICrop.stl"
        # stitch all pieces
        scan = reconstruct_pcd()
        # downsample/denoise
        scan = downsample_denoise(scan)
        o3d.io.write_point_cloud("reconstruction of phantom head.pcd", scan)
        # mri mesh to pcd (downsample)
        # o3d.visualization.draw_geometries([scan,pcd_coord])
        mri = mesh_to_pcd_downsample_mri(sourcePath)
        o3d.visualization.draw_geometries([mri, pcd_coord, scan])
        # mri = mri.transform(np.asarray([
        #     0, 0, 1, 0,
        #     0, 1, 0, 0,
        #     -1, 0, 0, 0,
        #     0, 0, 0, 1
        # ]).reshape(4, 4))
        # o3d.visualization.draw_geometries([mri,pcd_coord, scan])
        # # registration
        transformationMatrix = registration(mri, scan)
        # draw_registration_result(mri, scan, transformationMatrix)
        #
        print(transformationMatrix)
         # print(f"sent data:\n {transformationMatrix}")
        draw_registration_result(mri, scan, transformationMatrix)
    else:
        pcd=o3d.io.read_point_cloud('reconstruction of phantom head.pcd')
        o3d.visualization.draw_geometries([pcd])



