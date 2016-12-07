
#include "rotation_engine.hpp"

/*
 * Note that comments in this section are lacking. This needs to be updated
 * in the future. Please note that all inputs to any functions in this file
 * are defined to be in degrees.
 */

/*
psi = roll
phi = pitch
theta = yaw
*/

static constexpr double _PI_OVER_180 = 3.14159 / 180.0;
static constexpr double _180_OVER_PI = 180.0 / 3.14159 ;

double asind(double x)
{
    return _180_OVER_PI * asin(x);
}

double atan2d(double x, double y)
{
    return _180_OVER_PI * atan2(x, y);
}

double cosd(double x)
{
    return cos(x * _PI_OVER_180);
}

double sind(double x)
{
    return sin(x * _PI_OVER_180);
}

Matrix3d r3D(Vector3d omega)
{

    double psi    = omega(0),
           phi    = omega(1),
           theta  =  omega(2);


    Vector3d uz(0,0,1),
             uy(0,1,0),
             ux(1,0,0);

    Matrix3d tmp;
    Matrix3d Q,R;


    //yaw
    tmp <<    0, -uz(2),  uz(1),
          uz(2),      0, -uz(0),
         -uz(1),  uz(0),      0;

    Q = tmp*sind(theta) + (Matrix3d::Identity()-uz*uz.transpose())*cosd(theta) + uz*uz.transpose();


    uy = Q*uy; //new pitch vector
    ux = Q*ux; //new roll vector

    R = Q;


    //pitch
    tmp <<    0, -uy(2),  uy(1),
          uy(2),      0, -uy(0),
         -uy(1),  uy(0),      0;

    Q = tmp*sind(phi) + (Matrix3d::Identity()-uy*uy.transpose())*cosd(phi) + uy*uy.transpose();

    ux = Q*ux;

    R = Q*R;


    //roll
    tmp <<    0, -ux(2),  ux(1),
          ux(2),      0, -ux(0),
         -ux(1),  ux(0),      0;

    Q = tmp*sind(psi) + (Matrix3d::Identity()-ux*ux.transpose())*cosd(psi) + ux*ux.transpose();

    R = Q*R;

    return R;
}

//should be able to combine this function with r3D
Matrix3d r3Dv(Vector3d omega, MatrixXd uv)
{

    double psi   = omega(0),
           phi   = omega(1),
           theta =  omega(2);

    if(uv.size() == 3)
        uv = r3Dv(uv, Matrix3d::Identity());

    Vector3d ux = uv.col(0),
             uy = uv.col(1),
             uz = uv.col(2);

    Matrix3d tmp;
    Matrix3d Q,R;


    //yaw
    tmp <<    0, -uz(2),  uz(1),
          uz(2),      0, -uz(0),
         -uz(1),  uz(0),      0;

    Q = tmp*sind(theta) + (Matrix3d::Identity()-uz*uz.transpose())*cosd(theta) + uz*uz.transpose();


    uy = Q*uy; //new pitch vector
    ux = Q*ux; //new roll vector

    R = Q;


    //pitch
    tmp <<    0, -uy(2),  uy(1),
          uy(2),      0, -uy(0),
         -uy(1),  uy(0),      0;

    Q = tmp*sind(phi) + (Matrix3d::Identity()-uy*uy.transpose())*cosd(phi) + uy*uy.transpose();

    ux = Q*ux;

    R = Q*R;


    //roll
    tmp <<    0, -ux(2),  ux(1),
          ux(2),      0, -ux(0),
         -ux(1),  ux(0),      0;

    Q = tmp*sind(psi) + (Matrix3d::Identity()-ux*ux.transpose())*cosd(psi) + ux*ux.transpose();

    R = Q*R;

    R = R*uv;

    return R;
}

Vector3d ir3D(Matrix3d R)
{
    Vector3d ux, iO, uy;
    double psi, phi, theta, tmp;

    theta = atan2d(R(1,0),R(0,0));
            phi = -asind(R(2,0));
            psi = atan2d(R(2,1),R(2,2));


    iO = Vector3d(psi, phi, theta).real();

    return iO;
}

/*
 * This function was found to be copied from an online source on the
 * pseudo-inverse algorithm. The source can be found at the following link:
 *      https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
 */
MatrixXd pinv(const MatrixXd &a, double epsilon)
{
    Eigen::JacobiSVD<MatrixXd::PlainObject> svd(a,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *
        svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() >
            tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()
            * svd.matrixU().adjoint();
}
