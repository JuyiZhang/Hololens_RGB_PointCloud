import numpy as np
import open3d as o3d
import  time

def main():

    # load the scanned point cloud
    pcd=o3d.io.read_point_cloud('data/head_cropped.pcd')
    o3d.visualization.draw_geometries([pcd])

    tic = time.time()

    # we extract keypoints in the pcd
    # the keypoints can be used in our future registration method
    keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd )
    toc = 1000 * (time.time() - tic)
    print("ISS Computation took {:.0f} [ms]".format(toc))

    # setting colors for keypoints and other points
    colors_keypoints = np.zeros((243, 3))
    colors_keypoints[:, 0] = 2
    keypoints.colors = o3d.utility.Vector3dVector(colors_keypoints)

    colors_pcd = np.zeros((243, 3))
    colors_pcd[:, :] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors_pcd)


    # o3d.visualization.draw_geometries([pcd,keypoints])
    o3d.visualization.draw_geometries([ keypoints])



if __name__=='__main__':
    # demo_crop_geometry()
    main()