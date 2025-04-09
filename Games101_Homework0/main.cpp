#include<cmath>
#include<eigen3/Eigen/Core>
#include<iostream>

int main()
{
    Eigen::Vector3f p(2, 1, 1);
    auto rad = static_cast<float>(45.0 / 180 * EIGEN_PI);
    Eigen::Matrix3f r;
    r << cos(rad), -sin(rad), 0,
        sin(rad), cos(rad), 0,
        0, 0, 1;
    Eigen::Matrix3f t;
    t << 1, 0, 1,
        0, 1, 2,
        0, 0, 1;
    Eigen::Vector3f pNew = t * r * p;
    std::cout << pNew << '\n';
    return 0;
}
