import numpy as np
import open3d as o3d
from scipy.spatial import distance
from sklearn.neighbors import KDTree
import os





def crop():
    scanned = o3d.io.read_point_cloud('./data/evaluate/scanned.ply')
    transformed_mri = o3d.io.read_point_cloud('./data/evaluate/mri_regi_transformed.ply')

    # for each point in the scanned point cloud, compute its distance to the cloest point in the MRI point cloud
    dists = scanned.compute_point_cloud_distance(transformed_mri)
    dists = np.asarray(dists)

    # by this selection, we can remove points that are not in our target region (the head)
    # for example, all points below the head will be removed
    ind = np.where(dists < 0.015)[0]
    scanned_only_mri = scanned.select_by_index(ind)
    o3d.visualization.draw_geometries_with_editing([scanned_only_mri])
    # save file
    o3d.io.write_point_cloud("./data/evaluate/cropped_scanned.ply", scanned_only_mri)

def evaluate_landmarks():

    # MRI point cloud
    mri_pcd=o3d.io.read_point_cloud('./data/evaluate/mri_regi_transformed.ply')

    # create a point cloud container for points near the markers regions
    marker_pcd=o3d.geometry.PointCloud()
    # there are 7 markers, hence we need to read 7 pcd files
    markers=['./data/evaluate/markers/back_left.ply',
             './data/evaluate/markers/back_middle.ply',
             './data/evaluate/markers/back_right.ply',
             './data/evaluate/markers/front_left.ply',
             './data/evaluate/markers/front_middle.ply',
             './data/evaluate/markers/front_right.ply',
             './data/evaluate/markers/center.ply']

    # read each file, and add new points to marker_pcd
    for marker in markers:
        pcd=o3d.io.read_point_cloud(marker)
        marker_pcd=marker_pcd+pcd
        # o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([mri_pcd,marker_pcd])

    # for each point in marker_pcd, compute its distance to the closest point in the MRI pcd
    dists=marker_pcd.compute_point_cloud_distance(mri_pcd)
    dists=np.asarray(dists)
    print('marker-based distance:',dists.mean())

def evaluate_global():
    # the global evaluation metrics
    # just load the scanned pcd and the MRI pcd
    # for each point in the scanned pcd, compute its distance to the closest point in the MRI pcd
    scanned = o3d.io.read_point_cloud('./data/evaluate/cropped_scanned.ply')
    transformed_mri = o3d.io.read_point_cloud('./data/evaluate/mri_regi_transformed.ply')
    dists = scanned.compute_point_cloud_distance(transformed_mri)
    dists = np.asarray(dists)
    print('global distance:',dists.mean())

def visualize_registration_result():
    scanned = o3d.io.read_point_cloud('./data/evaluate/cropped_scanned.ply')
    transformed_mri = o3d.io.read_point_cloud('./data/evaluate/mri_regi_transformed.ply')
    o3d.visualization.draw_geometries([scanned,transformed_mri])


def main():

    # function to crop the reconstruction of the phantom head ( the scanned point cloud)
    crop()

    # function to visualize our regisration result
    visualize_registration_result()

    evaluate_landmarks()
    evaluate_global()





if __name__=='__main__':
    main()

