import numpy as np
import open3d as o3d
from random import sample
import os


def reconstruct_pcd():
    # convert npy to pcd
    path = "./data/PointCloudCapture/"
    pcds = []
    for file in os.listdir(path):
        data = np.load(path + file)
        data[:, 0] = -data[:, 0]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data)
        pcds.append(pcd)

    # overlay all the pcds
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcd_combined += pcds[point_id]
    return pcd_combined


def downsample_denoise(pcd):
    num_pts = np.asarray(pcd.points).shape[0]

    # random downsample
    downsampled_pts = sample(range(num_pts), int(round(num_pts / 1)))
    if int(round(num_pts / 50)) < 3000:
        print("too few points, sample more data")
    # pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[downsampled_pts,:])

    # denoise
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.5)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.5)

    # voxel downsample
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    o3d.io.write_point_cloud("pcd_combined.pcd", pcd)
    num_pts = np.asarray(pcd.points).shape[0]
    print('Scanned pcd has ' + str(num_pts) + ' points')
    return pcd


def mesh_to_pcd_downsample_mri(path):
    mesh = o3d.io.read_triangle_mesh(path)
    if path=="./phantomHeadCrop.stl":
        mesh = mesh.scale(0.001, [0, 0, 0])
    if path == "./patientMRI.stl" or path == "./patientMRICrop.stl" :
        mesh = mesh.scale(0.001, [0, 0, 0])
    pcd = mesh.sample_points_uniformly(number_of_points=10000)
    pcd = pcd.voxel_down_sample(voxel_size=0.005)
    # if path == "./realPatient.stl":
    #     pcd=pcd.transform(np.asarray([
    #         1,0,0,0,
    #         0,0,1,0,
    #         0,-1,0,0,
    #         0,0,0,1
    #     ]).reshape(4,4))
    pcd.paint_uniform_color([1, 0.706, 0])
    return pcd
