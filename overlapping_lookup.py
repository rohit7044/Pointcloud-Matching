import open3d as o3d
import numpy as np
from utils import visualize_pointclouds


def non_overlapping_lookup(source, target, trans_init):
    # Transform the source point cloud
    source_transformed = source.transform(trans_init)


    # Build a KD-Tree for the target point cloud to find nearest neighbors
    target_kdtree = o3d.geometry.KDTreeFlann(target)

    # Calculate the threshold
    threshold = 0.01

    # Initialize lists to store overlapping and non-overlapping points
    overlap_points = []
    non_overlap_points = []

    # Iterate over each point in the transformed source point cloud
    for point in source_transformed.points:
        # Find the nearest point in the target point cloud
        [_, idx, dists] = target_kdtree.search_knn_vector_3d(point, 1)  # 1 nearest neighbor

        # If the distance to the nearest neighbor is greater than the threshold, it's non-overlapping
        if dists[0] > threshold:
            non_overlap_points.append(point)
        else:
            overlap_points.append(point)

    # Create new point clouds for overlapping and non-overlapping points
    overlap_pcd = o3d.geometry.PointCloud()
    non_overlap_pcd = o3d.geometry.PointCloud()

    # Assign points to the new point clouds
    overlap_pcd.points = o3d.utility.Vector3dVector(np.array(overlap_points))
    non_overlap_pcd.points = o3d.utility.Vector3dVector(np.array(non_overlap_points))

    # Set colors for visualization
    overlap_pcd.paint_uniform_color([0, 1, 0])  # Green for overlapping points
    non_overlap_pcd.paint_uniform_color([1, 0, 0])  # Red for non-overlapping points

    visualize_pointclouds([overlap_pcd, non_overlap_pcd])



