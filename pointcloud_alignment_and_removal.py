import argparse as args
import numpy as np
from local_pointcloud_registration import compute_local_transformation
from global_pointcloud_registration import compute_global_transformation
from utils import read_pointcloud, visualize_pointcloud, draw_registration_result, decompose_transformation_matrix, preprocess_point_cloud
from overlapping_lookup import non_overlapping_lookup





if __name__ == "__main__":
    parser = args.ArgumentParser()
    parser.add_argument("point_cloud_path1", type=str, help="Path to the first point cloud file")
    parser.add_argument("point_cloud_path2", type=str, help="Path to the second point cloud file")
    args = parser.parse_args()

    point_cloud1_path = args.point_cloud_path1
    point_cloud2_path = args.point_cloud_path2

    point_cloud1 = read_pointcloud(point_cloud1_path)
    point_cloud2 = read_pointcloud(point_cloud2_path)

    voxel_size = 0.5

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

    # Optional: Find the non-overlapping points

    # For using predefined transformation matrix
    # non_overlapping_lookup(point_cloud1, point_cloud2, reg_local.transformation)

    # reg_local = np.array([[-9.43697404e-01, -1.86913863e-02, -3.30281458e-01, -4.31757612e+00],
    #                       [-6.56189190e-03,  9.99263717e-01, -3.78016493e-02,  2.35961511e-01],
    #                       [ 3.30744843e-01, -3.35060471e-02, -9.43125227e-01,  7.70472897e+00],
    #                       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    # For using custom transformation matrix
    # non_overlapping_lookup(point_cloud1_down, point_cloud2_down, reg_local)
