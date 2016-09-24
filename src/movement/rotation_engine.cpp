
#include "rotation_engine.hpp"

/*
psi = roll
phi = pitch
theta = yaw
*/

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

    Q = tmp*sin(theta) + (Matrix3d::Identity()-uz*uz.transpose())*cos(theta) + uz*uz.transpose();


    uy = Q*uy; //new pitch vector
    ux = Q*ux; //new roll vector

    R = Q;


    //pitch
    tmp <<    0, -uy(2),  uy(1),
          uy(2),      0, -uy(0),
         -uy(1),  uy(0),      0;

    Q = tmp*sin(phi) + (Matrix3d::Identity()-uy*uy.transpose())*cos(phi) + uy*uy.transpose();

    ux = Q*ux;

    R = Q*R;


    //roll
    tmp <<    0, -ux(2),  ux(1),
          ux(2),      0, -ux(0),
         -ux(1),  ux(0),      0;

    Q = tmp*sin(psi) + (Matrix3d::Identity()-ux*ux.transpose())*cos(psi) + ux*ux.transpose();

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

    Q = tmp*sin(theta) + (Matrix3d::Identity()-uz*uz.transpose())*cos(theta) + uz*uz.transpose();


    uy = Q*uy; //new pitch vector
    ux = Q*ux; //new roll vector

    R = Q;


    //pitch
    tmp <<    0, -uy(2),  uy(1),
          uy(2),      0, -uy(0),
         -uy(1),  uy(0),      0;

    Q = tmp*sin(phi) + (Matrix3d::Identity()-uy*uy.transpose())*cos(phi) + uy*uy.transpose();

    ux = Q*ux;

    R = Q*R;


    //roll
    tmp <<    0, -ux(2),  ux(1),
          ux(2),      0, -ux(0),
         -ux(1),  ux(0),      0;

    Q = tmp*sin(psi) + (Matrix3d::Identity()-ux*ux.transpose())*cos(psi) + ux*ux.transpose();

    R = Q*R;

    R = R*uv;

    return R;
}

Vector3d ir3D(Matrix3d R)
{
    Vector3d ux, iO, uy;
    double psi, phi, theta, tmp;


/*		ux = R*Vector3d(1, 0, 0);
    theta = atan2(ux(1), ux(0));
    phi = -asin(ux(2));

    psi = atan2(R(2,1),R(2,2));

    uy = r3D(Vector3d(0, phi, theta))*Vector3d(0,1,0);

    tmp = (R*Vector3d(0,1,0)).transpose()*uy;
    if (abs(tmp) >= 1)
        psi = 0;
    else
        psi = acos(tmp);


iO = Vector3d(psi, phi, theta).real();

            //if((R-r3D(Vector3d(-psi, phi, theta))).norm() <= (R-r3D(iO)).norm())
                    //iO(0) = -psi;


*/


    theta = atan2(R(1,0),R(0,0));
            phi = -asin(R(2,0));
            psi = atan2(R(2,1),R(2,2));


    iO = Vector3d(psi, phi, theta).real();

    return iO;
}

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
