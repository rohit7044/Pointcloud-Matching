# Pointcloud Matching using Open3D

Explanation:
The main goal:
Align the point clouds and retrieve the Translation Matrix, Rotation Matrix, Scaling Matrix

Optional goal:
Extract the non-overlapping data

To run the code:
```commandline
python main.py --pc1 "point_cloud_1.ply" --pc2 "point_cloud_2.ply" --voxel_size 0.05 --threshold 0.4
```
The approach:
1. Preprocessed and downsampled the pointclouds
2. Initiate Global Registration to get an initial transformation matrix
3. Pass the global transformation matrix to ICP Registration for next step
4. Retrieve the transformation matrix and decompose to retrieve the Translation Matrix, Rotation Matrix, Scaling Matrix

The Drawback:

I feel the alignment is not always accurate. When I see the aligned pointcloud, it looks like it needs to be rotated another 90º. And since Global transformation matrix is not aligning it right, the ICP though gets the initial matrix is also struggling with it. The RMSE though comes <3, it struggles with fitting <5.
Another drawback is lack of difference of points in both the pointclouds. One pointcloud has more points thatn the other so alignment based on points alone is a challenge.




The viable Solution: 

Having reference points: Having reference points before aligning might help us with the rotation as well as excess pointcloud issue. 

Better algorithm: Kabsch Algorithm might work better but I didn't test it yet to come to a conclusion.

Lower voxel size gives better convergence at the cost of computation time.
