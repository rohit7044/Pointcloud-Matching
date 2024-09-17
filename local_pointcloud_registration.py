import open3d as o3d
from utils import preprocess_point_cloud


def compute_local_transformation(point_cloud1_down, point_cloud2_down,trans_init, threshold):
    reg_p2p = o3d.pipelines.registration.registration_icp(
        point_cloud1_down, point_cloud2_down, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    return reg_p2p