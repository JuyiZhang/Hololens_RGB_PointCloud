import open3d as o3d
import time

# compute geometry features, namely normals, fpfh
def preprocess_point_cloud(pcd, voxel_size,reorientNormal=False):
    radius_normal = voxel_size * 2
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    if reorientNormal==True:
        o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd)

    radius_feature = voxel_size * 5

    # fpfh feature
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd, pcd_fpfh


def prepare_dataset(voxel_size, source, target):
    target.estimate_normals()

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size,reorientNormal=True)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, False,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(
                3.1415926/180.0*10.0),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(5000000, 0.999999))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_fast):
    distance_threshold = voxel_size * 0.4
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_fast.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def registration(source, target):
    voxel_size = 0.005  # same as preprocess
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, source, target)
    start = time.time()
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    print("Global registration took %.3f sec.\n" % (time.time() - start))

    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                     voxel_size, result_fast)
    print(result_icp)
    transformationMatrix = result_icp.transformation
    return transformationMatrix