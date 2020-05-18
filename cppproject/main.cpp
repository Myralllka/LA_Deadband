#include <iostream>
#include <Eigen/Dense>

#define FACTOR 3

Eigen::Vector3d check_reference(Eigen::Vector3d target, Eigen::Vector3d reference, Eigen::Vector3d uav_odom){
    auto subspace = target - uav_odom;

    return reference;
}

int main() {
//    std::cout << "Hello, World!" << std::endl;
    Eigen::Vector3d tar(2,3,5);
    Eigen::Vector3d ref(1,2,3);
    Eigen::Vector3d odom(1,2,3);
    std::cout << check_reference(tar, ref, odom) << std::endl;
}