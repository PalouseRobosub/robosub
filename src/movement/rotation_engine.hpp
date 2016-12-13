#ifndef ROTATIONENGINE_HPP
#define ROTATIONENGINE_HPP

#include <eigen3/Eigen/Dense>
#include <limits>

using namespace Eigen;

Matrix3d r3D(Vector3d omega);
Matrix3d r3Dv(Vector3d omega, MatrixXd uv);
Vector3d ir3D(Matrix3d R);
MatrixXd pinv(const MatrixXd &a,
              double epsilon = std::numeric_limits<double>::epsilon());

#endif
