#include <iostream>
#include <Eigen/Dense>

#define FACTOR 3

Eigen::Vector3d check_reference(Eigen::Vector3d target, Eigen::Vector3d reference, Eigen::Vector3d uav_odom) {
    auto subspace = target - uav_odom;
    auto projection_matrix = (subspace * subspace.transpose()) / subspace.dot(subspace);
    auto orthogonal_projection_matrix = Eigen::MatrixXd::Identity(3, 3) - projection_matrix;
    auto projection = projection_matrix * (uav_odom - reference);
    auto length = std::sqrt((uav_odom - reference).dot(uav_odom - reference));
    if (length < FACTOR) {
        return (orthogonal_projection_matrix * (reference - uav_odom)) + uav_odom;
    } else return reference;
}

int main() {
//    std::cout << "Hello, World!" << std::endl;
    Eigen::Vector3d tar(10, 10, 0);
    Eigen::Vector3d ref(10, 5, 0);
    Eigen::Vector3d odom(11, 5, 0);
    std::cout << check_reference(tar, ref, odom) << std::endl;
}