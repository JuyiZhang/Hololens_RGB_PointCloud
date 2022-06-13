import numpy as np
import open3d as o3d
from sklearn.neighbors import KDTree
import os

def visualize(xyz,file_path):

    # create a pcd object
    pcd = o3d.geometry.PointCloud()
    # add points from xyz (numpy array)
    pcd.points = o3d.utility.Vector3dVector(xyz)
    # save the created pcd as a file
    o3d.io.write_point_cloud(file_path, pcd)

    #load and visualize
    pcd_load = o3d.io.read_point_cloud(file_path)
    o3d.visualization.draw_geometries([pcd_load])

def find_K_nearest_neighbours(xyz,K):

    # brute force method to compute top-K nearest neighbours
    # distance_matrix = distance.squareform(distance.pdist(xyz))
    # index=np.argsort(distance_matrix,axis=1)

    # KD Tree based method to compute top-K nearest neighbours
    # more efficient than the brute force method
    tree=KDTree(xyz)
    neighbours_dist,neighbours_indx=tree.query(xyz,k=K+1)
    return neighbours_dist[:,1:],neighbours_indx[:,1:]



def denoise(xyz):
    # find top-K nearest neighbours
    neighbours_dist,neighbours_indx=find_K_nearest_neighbours(xyz,K=1000)

    #compute the mean distance between each point and its K nearest neighbours
    # The distance vector D in our report
    mean_neighbours_dist=neighbours_dist.mean(axis=1)

    # mean(D)
    global_mean=mean_neighbours_dist.mean()
    # var(D)
    global_var=mean_neighbours_dist.var()
    alpha=0.1
    threshold=global_mean+alpha*global_var
    return xyz[mean_neighbours_dist<threshold]




def main():

    # list of files to be denoised

    xyz_list=['./data/denoising.npy',
              # './data/1648994242_pc_right.ndarray.npy',
              # './data/1648994346_pc_rightback34.ndarray.npy',
              # './data/pcd/1652898244971_pc.ndarray.npy',
              # './data/pcd/1652898245148_pc.ndarray.npy'
              ]
    # xyz_list=[os.path.join('./data/PointCloudCapture',f)
    #           for f in os.listdir('./data/PointCloudCapture') if os.path.isfile(os.path.join(
    #     './data/PointCloudCapture',f
    # ))]

    # denoise each point cloud
    # then save as a ply file
    for i,file in enumerate(xyz_list):
        print(f'{i}/{len(xyz_list)}')
        xyz=np.load(file)
        visualize(xyz,
                  file_path=file.replace('PointCloudCapture', 'PointCloudCapture_denoised').replace('npy', 'ply'))
        if xyz.shape[0]<1:
            continue
        # visualize(xyz,file_path=file.replace('pcd','pcd_denoise').replace('npy','ply'))
        xyz_denoised=denoise(xyz)
        visualize(xyz_denoised, file_path=file.replace('PointCloudCapture', 'PointCloudCapture_denoised').replace('npy','ply'))
    # xyz=np.load(xyz_list[3])
    # visualize(xyz,file_path='./data/test.ply')
    # xyz_denoised=denoise(xyz)
    # print(f'xyz shape: {xyz.shape}, xyz_denoised shape: {xyz_denoised.shape}')
    # visualize(xyz_denoised,file_path='./data/denoisez.ply')


if __name__=='__main__':
    main()


# def custom_draw_geometry_with_key_callback(pcd):
#     def change_background_to_black(vis):
#         opt = vis.get_render_option()
#         opt.background_color = np.asarray([0, 0, 0])
#         return False
#     def load_render_option(vis):
#         vis.get_render_option().load_from_json(
#                 "../../TestData/renderoption.json")
#         return False
#     def capture_depth(vis):
#         depth = vis.capture_depth_float_buffer()
#         plt.imshow(np.asarray(depth))
#         plt.show()
#         return False
#     def capture_image(vis):
#         image = vis.capture_screen_float_buffer()
#         plt.imshow(np.asarray(image))
#         plt.show()
#         return False
#     key_to_callback = {}
#     key_to_callback[ord("K")] = change_background_to_black
#     key_to_callback[ord("R")] = load_render_option
#     key_to_callback[ord(",")] = capture_depth
#     key_to_callback[ord(".")] = capture_image
#     draw_geometries_with_key_callbacks([pcd], key_to_callback)
#
# custom_draw_geometry_with_key_callback(point_cloud)