import numpy as np

import open3d as o3d
import copy

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      # zoom=0.4559,
                                      # front=[0.6452, -0.3036, -0.7011],
                                      # lookat=[1.9892, 2.0208, 1.8945],
                                      # up=[-0.2779, -0.9482, 0.1556]
                                      )

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


# the code for preparing the dataset
# This is from the Open3D tutorial!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# This is from the Open3D tutorial!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# This is from the Open3D tutorial!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def prepare_dataset(voxel_size,source,target):

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):

    # we apply the fast global registration from the paper
    # Q.-Y. Zhou, J. Park, and V. Koltun, Fast Global Registration, ECCV, 2016.
    distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def stl_to_pcd(stl_file):

    pcd_file=stl_file.replace('.stl','.pcd')
    # read the MRI mesh from the .stl file
    mesh=o3d.io.read_triangle_mesh(stl_file)

    # the MRI data and the scanned data have different scales
    # hence, rescaling is required here
    mesh.scale(0.001,center=mesh.get_center())

    # move the mesh data to a position near our scanned data
    # so that we can visualize them in the same window
    mesh=mesh.translate((3.45215046,111.07869855,145.31498048))
    R=mesh.get_rotation_matrix_from_xyz((-np.pi/2,0,np.pi))
    mesh=mesh.rotate(R)

    # convert the mesh to a point cloud by sampling
    pointcloud=mesh.sample_points_poisson_disk(30000)
    o3d.io.write_point_cloud(pcd_file,pointcloud)


def main():
    # the given MRI data is a .stl file
    # we need to convert it to a point cloud, namely a .pcd file
    reference_stl_file = './data/BTI_DemoHead/skin/Segmentation_Skin.stl'
    stl_to_pcd(reference_stl_file)
    reference_pcd_file = reference_stl_file.replace('.stl', '.pcd')
    reference_point_cloud=o3d.io.read_point_cloud(reference_pcd_file)

    # load the scanned point cloud
    # the reconstruction is made with un-matching frames and matching frames, for the purpose of robust experiment
    scanned_head_pcd_file= 'data/scanned_head_robust_experiment.pcd'
    scanned_point_cloud=o3d.io.read_point_cloud(scanned_head_pcd_file)

    # visualize the reconstruction of the phantom head
    # we can see that there is some error around the nose regions due to the un-matching frames
    o3d.visualization.draw_geometries([scanned_point_cloud])

    # visualize both the scanned pcd and the MRI pcd
    o3d.visualization.draw_geometries([scanned_point_cloud, reference_point_cloud])

    #global registration
    voxel_size = 0.005  # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size,source=scanned_point_cloud,target=reference_point_cloud)
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    print(result_fast.transformation)
    draw_registration_result(source_down, target_down, result_fast.transformation)
    trans_init = result_fast.transformation



    # local registration
    # the global registration result is the initial point of the local registration
    threshold=0.02
    print("Initial alignment")
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    print("ICP alignment")
    draw_registration_result(source, target, reg_p2p.transformation)




if __name__=='__main__':
    main()