import argparse as args
import numpy as np
from local_pointcloud_registration import compute_local_transformation
from global_pointcloud_registration import compute_global_transformation
from utils import read_pointcloud, visualize_pointcloud, draw_registration_result, decompose_transformation_matrix, preprocess_point_cloud
from overlapping_lookup import non_overlapping_lookup


if __name__ == "__main__":
    parser = args.ArgumentParser()
    parser.add_argument("--pc1", dest="point_cloud_path1", type=str, help="Path to the first point cloud file")
    parser.add_argument("--pc2", dest="point_cloud_path2", type=str, help="Path to the second point cloud file")
    parser.add_argument("--voxel_size", dest = "voxel_size", type=float, help="Voxel size for downsampling")
    parser.add_argument("--threshold", dest = "threshold", type=float, help="Threshold value for ICP registration")
    args = parser.parse_args()

    # Now you can access the parsed values as follows:
    point_cloud1_path = args.point_cloud_path1
    point_cloud2_path = args.point_cloud_path2
    voxel_size = args.voxel_size
    threshold = args.threshold

    point_cloud1 = read_pointcloud(point_cloud1_path)
    point_cloud2 = read_pointcloud(point_cloud2_path)

    point_cloud1_down, source_fpfh = preprocess_point_cloud(point_cloud1, voxel_size)
    point_cloud2_down, target_fpfh = preprocess_point_cloud(point_cloud2, voxel_size)

    print("\n:: Performing RANSAC registration on downsampled point clouds.")
    reg_global = compute_global_transformation(point_cloud2_down, target_fpfh, point_cloud1_down, source_fpfh,voxel_size)

    print(reg_global.transformation)
    print(reg_global)

    print("\n:: ICP registration on original point clouds.")

    reg_local = compute_local_transformation(point_cloud2_down, point_cloud1_down,reg_global.transformation, threshold=0.4)

    print(reg_local.transformation)
    print(reg_local)

    draw_registration_result(point_cloud2_down, point_cloud1_down, reg_local.transformation)

    translation, rotation, scale = decompose_transformation_matrix(reg_local.transformation)

    print("Translation:")
    print(translation)

    print("Rotation:")
    print(rotation)

    print("Scale:")
    print(scale)

    # Optional: Find the non-overlapping points Uncomment to make it work

    # For using predefined transformation matrix
    # non_overlapping_lookup(point_cloud2_down, point_cloud1_down, reg_local.transformation)

    # For using custom transformation matrix. This one works!
    # reg_local = np.array([[-9.41759366e-01, -1.78947952e-02, -3.35811067e-01, -4.23329280e+00],
    #                       [-5.58928585e-03,  9.99278177e-01, -3.75750451e-02,  2.22416883e-01],
    #                       [ 3.36241069e-01, -3.35097066e-02, -9.41179602e-01,  7.80178598e+00],
    #                       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    # non_overlapping_lookup(point_cloud2_down, point_cloud1_down, reg_local)
