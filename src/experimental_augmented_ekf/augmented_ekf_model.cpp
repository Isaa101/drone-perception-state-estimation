/*
 * Recovered experimental source from a later team integration stage of a university robotics project.
 * The augmented EKF did not validate reliably in the final integration.
 * Implementation logic below is preserved as recovered; see docs/failure_analysis.md and SOURCE_RECOVERY.md.
 */


#include <ekf_model.h>

namespace ekf_imu_vision {
  inline double clamp_euler_angle(double theta)
{
    double result = atan2 (sin(theta),cos(theta)); 
    return result;
}

Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the model xdot = f(x,u,n)

  Vec15 xdot;

  //  1: x0:2 ~ x, y, z """
  //  2: x3:5 ~ phi theta psi """
  //  3: x6:8 ~ vx vy vz """
  //  4: x9:11 ~ bgx bgy bgz """
  //  5: x12:14 ~  bax bay baz """

  // u0:2 wmx, wmy, wmz
  // u3:5 amx, amy, amz

  // n0:2 ngx, ngy, ngz
  // n3:5 nax, nay, naz
  // n6:8 nbgx, nbgy, nbgz
  // n9:11 nbax, nbay, nbaz

  //state components
  Vec3 position = x.segment<3>(0);
  Vec3 euler_angles = x.segment<3>(3);
  Vec3 velocity = x.segment<3>(6);
  Vec3 gyro_bias = x.segment<3>(9);
  Vec3 accel_bias = x.segment<3>(12);

  double phi = euler_angles(0);
  double theta = euler_angles(1);
  double psi = euler_angles(2);

  //input components
  Vec3 ang_vel = u.segment<3>(0);
  Vec3 accel = u.segment<3>(3);

  //noise components
  Vec3 gyro_noise = n.segment<3>(0);
  Vec3 accel_noise = n.segment<3>(3);
  Vec3 gyro_bias_noise = n.segment<3>(6);
  Vec3 accel_bias_noise = n.segment<3>(9);

  //trigonometric terms
  double c_phi = cos(phi),   s_phi = sin(phi);
  double c_theta = cos(theta), s_theta = sin(theta);
  double c_psi = cos(psi),   s_psi = sin(psi);

  //rotation matrices
  Mat3x3 G;
  G << c_theta, 0, -c_phi * s_theta,
      0,      1,  s_phi,
      s_theta, 0,  c_theta * c_phi;

  Mat3x3 R;
  R << c_psi * c_theta - s_phi * s_psi * s_theta, -c_phi * s_psi, c_psi * s_theta + c_theta * s_phi * s_psi,
      c_theta * s_psi + c_psi * s_phi * s_theta,  c_phi * c_psi, s_psi * s_theta - c_psi * s_phi * c_theta,
      -c_phi * s_theta,                            s_phi,         c_phi * c_theta;

  Mat3x3 G_inv = G.inverse();

  //derivatives
  xdot.segment<3>(0) = velocity;
  xdot.segment<3>(3) = G_inv * (ang_vel - gyro_bias - gyro_noise);
  xdot.segment<3>(6) = Vec3(0, 0, -9.81) + R * (accel - accel_bias - accel_noise);
  xdot.segment<3>(9) = gyro_bias_noise;
  xdot.segment<3>(12) = accel_bias_noise;


  //clamp angles
  xdot(3) = atan2(sin(xdot(3)), cos(xdot(3)));
  xdot(4) = atan2(sin(xdot(4)), cos(xdot(4)));
  xdot(5) = atan2(sin(xdot(5)), cos(xdot(5)));


  return xdot;
}

Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt original state df/dx

  Vec3 ang_vel = u.segment<3>(0);
  Vec3 accel   = u.segment<3>(3);

  double phi   = x(3);
  double theta = x(4);
  double psi   = x(5);

  Vec3 gyro_bias  = x.segment<3>(9);
  Vec3 accel_bias = x.segment<3>(12);

  //trigonometric functions
  double c_phi = cos(phi),   s_phi = sin(phi);
  double c_theta = cos(theta), s_theta = sin(theta);
  double c_psi = cos(psi),   s_psi = sin(psi);

  //rotation matrices
  Mat3x3 G;
  G << c_theta, 0, -c_phi * s_theta,
      0,      1,  s_phi,
      s_theta, 0,  c_phi * c_theta;

  Mat3x3 R;
  R << c_psi * c_theta - s_phi * s_psi * s_theta, -c_phi * s_psi, c_psi * s_theta + c_theta * s_phi * s_psi,
      c_theta * s_psi + c_psi * s_phi * s_theta,  c_phi * c_psi, s_psi * s_theta - c_psi * s_phi * c_theta,
      -c_phi * s_theta,                            s_phi,         c_phi * c_theta;

  Mat3x3 G_inv = G.inverse();

  //jacobian
  Mat15x15 At = Mat15x15::Zero();
  At.block<3,3>(0,6) = Mat3x3::Identity();

  //A_x2_x2 (Jacobian of Euler angle dynamics)
  const double wx = ang_vel(0), wy = ang_vel(1), wz = ang_vel(2);
  const double bgx = x(9), bgy = x(10), bgz = x(11);

  const double cphi = cos(phi), sphi = sin(phi);
  const double ctheta = cos(theta), stheta = sin(theta);
  const double ctheta2 = ctheta*ctheta, stheta2 = stheta*stheta;
  const double denom = ctheta2 + stheta2;  

  
  Mat3x3 A_x2_x2 = Mat3x3::Zero();
  A_x2_x2(0,1) = (stheta*(wx - bgx) - ctheta*(wz - bgz))/denom;
  A_x2_x2(1,0) = (cphi*ctheta*(wz - bgz) - cphi*stheta*(wx - bgx))/(cphi*denom) + (stheta*sphi*(wz - bgz) - stheta*sphi*(wx - bgx))/(cphi*denom);
  A_x2_x2(1,1) = -(sphi*stheta*(wz - bgz) + ctheta*sphi*(wx - bgx))/(cphi*denom);
  A_x2_x2(2,0) = (stheta*sphi*(wx - bgx) - ctheta*sphi*(wz - bgz))/(cphi*denom);
  A_x2_x2(2,1) = (ctheta*(wx - bgx) + stheta*(wz - bgz))/(cphi*denom);

  At.block<3,3>(3,3) = A_x2_x2;

  Mat3x3 R_dot, skew_matrix;
  skew_matrix << 0, -ang_vel(2), ang_vel(1),
            ang_vel(2), 0, -ang_vel(0),
            -ang_vel(1), ang_vel(0), 0;
  R_dot = R * skew_matrix;

  At.block<3,3>(6,3) = R_dot;

  At.block<3,3>(3,9) = -G_inv;
  At.block<3,3>(6,12) = -R;

  return At;

}

Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt noise df/dn

  Vec3 ang_vel = u.segment<3>(0);   
  Vec3 accel   = u.segment<3>(3);

  double phi   = x(3);
  double theta = x(4);
  double psi   = x(5);

  double c_phi = cos(phi),   s_phi = sin(phi);
  double c_theta = cos(theta), s_theta = sin(theta);
  double c_psi = cos(psi),   s_psi = sin(psi);

  Mat3x3 G;
  G << c_theta, 0, -c_phi * s_theta,
      0,      1,  s_phi,
      s_theta, 0,  c_theta * c_phi;

  Mat3x3 R;
  R << c_psi * c_theta - s_phi * s_psi * s_theta, -c_phi * s_psi, c_psi * s_theta + c_theta * s_phi * s_psi,
        c_theta * s_psi + c_psi * s_phi * s_theta,  c_phi * c_psi, s_psi * s_theta - c_psi * s_phi * c_theta,
      -c_phi * s_theta,                            s_phi,         c_phi * c_theta;
  
  Mat3x3 G_inv = G.inverse();


  Mat15x12 Ut = Mat15x12::Zero(15,12);
  Ut.block<3,3>(3,0) << -G_inv;
  Ut.block<3,3>(6,3) << -R;
  Ut.block<6,6>(9,6) << Mat6x6::Identity(6,6);
  return Ut;
}

/* ============================== model of PnP ============================== */

Vec6 modelG1(const Vec15& x, const Vec6& v) {

  // TODO: the down looking camera
  // FIXME: assume measurement of body in world frame. x is the measurement from PnP
  // return the model g(x,v), where x = x_origin

  Vec6 zt;

  zt=x.segment(0,6);

  return zt;
}

Mat6x15 jacobiG1x(const Vec15& x, const Vec6& v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = x_origin

  Mat6x15 Ct;

  Ct.setZero();
  Ct.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

  return Ct;
}

Mat6x6 jacobiG1v(const Vec15& x, const Vec6& v) {

  // TODO;
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;

  I6.setIdentity();

  return I6;
}

/* ============================== model of stereo VO relative pose ============================== */

//Vec6 modelG2(const Vec21& x, const Vec6& v) {
//
  // TODO
  // return the model g(x,v), where x = (x_origin, x_augmented)

//  Vec6 zt;
//  Mat3x3 R_qk=rpy2Rotmat(x(18),x(19),x(20));

//  zt.segment(0,3) = R_qk.transpose() * (x.segment(0, 3)- x.segment(15, 3));
//  zt.segment(3,3) = rot2Euler(R_qk.transpose() * rpy2Rotmat(x(3),x(4),x(5)));
  
//  lampEuler(zt(3));
//  clampEuler(zt(4));
//  clampEuler(zt(5));

//  return zt;
//}
Vec6 modelG2(const Vec21 &state, const Vec6 &v)
  {

    // TODO
    // return the model g(x,v), where x = (x_origin, x_augmented)

    Vec6 zt;
    Eigen::Quaterniond q_wk, q_wb;
    q_wb = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY());

    q_wk = Eigen::AngleAxisd(state(20), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(state(18), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(state(19), Eigen::Vector3d::UnitY());

    Eigen::Matrix3d R_wk = q_wk.matrix();

    Eigen::Matrix3d R_kb = R_wk.transpose() * q_wb.matrix();

    zt.segment<3>(0) = R_wk.transpose() * (state.head(3) - state.segment<3>(15));

    zt(3) = asin(R_kb(2, 1));
    zt(4) = atan2(-R_kb(2, 0), R_kb(2, 2));
    zt(5) = atan2(-R_kb(0, 1), R_kb(1, 1));

    return zt;
  }
Mat6x21 jacobiG2x(const Vec21 &state, const Vec6 &noise)
  {

    // TODO
    // return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)
    double x_b = state(0), y_b = state(1), z_b = state(2);
    double phi_b = state(3), theta_b = state(4), psi_b = state(5);
    double x_k = state(15), y_k = state(16), z_k = state(17);
    double phi_k = state(18), theta_k = state(19), psi_k = state(20);

    double cphik = cos(phi_k), sphik = sin(phi_k);
    double cthetak = cos(theta_k), sthetak = sin(theta_k);
    double cpsik = cos(psi_k), spsik = sin(psi_k);

    double cphib = cos(phi_b), sphib = sin(phi_b);
    double cthetab = cos(theta_b), sthetab = sin(theta_b);
    double cpsib = cos(psi_b), spsib = sin(psi_b);
    
    Mat6x21 Ct = Mat6x21::Zero();

    // Helper deltas
    double dx = x_b - x_k;
    double dy = y_b - y_k;
    double dz = z_b - z_k;

    // Reuse for Ct(0,:) and Ct(2,:)
    double common0 = cpsik * cthetak - sphik * spsik * sthetak;
    double common1 = cthetak * spsik + cpsik * sphik * sthetak;
    double common2 = -cphik * sthetak;

    // Row 0
    Ct(0, 0) = common0;
    Ct(0, 1) = common1;
    Ct(0, 2) = common2;
    Ct(0, 15) = -common0;
    Ct(0, 16) = -common1;
    Ct(0, 17) = -common2;
    Ct(0, 18) = sphik * sthetak * dz + cphik * cpsik * sthetak * dy - cphik * spsik * sthetak * dx;
    Ct(0, 19) = -(cpsik * sthetak + cthetak * sphik * spsik) * dx - (spsik * sthetak - cpsik * cthetak * sphik) * dy - cphik * cthetak * dz;
    Ct(0, 20) = -(cthetak * spsik + cpsik * sphik * sthetak) * dx + (cpsik * cthetak - sphik * spsik * sthetak) * dy;

    // Row 1
    Ct(1, 0) = -cphik * spsik;
    Ct(1, 1) = cphik * cpsik;
    Ct(1, 2) = sphik;
    Ct(1, 15) = -Ct(1, 0);
    Ct(1, 16) = -Ct(1, 1);
    Ct(1, 17) = -Ct(1, 2);
    Ct(1, 18) = cphik * dz - cpsik * sphik * dy + spsik * sphik * dx;
    Ct(1, 20) = -cphik * cpsik * dx - cphik * spsik * dy;

    // Row 2
    Ct(2, 0) = cpsik * sthetak + cthetak * sphik * spsik;
    Ct(2, 1) = spsik * sthetak - cpsik * cthetak * sphik;
    Ct(2, 2) = cphik * cthetak;
    Ct(2, 15) = -Ct(2, 0);
    Ct(2, 16) = -Ct(2, 1);
    Ct(2, 17) = -Ct(2, 2);
    Ct(2, 18) = -cthetak * sphik * dz - cphik * cpsik * cthetak * dy + cphik * cthetak * spsik * dx;
    Ct(2, 19) = (cpsik * cthetak - sphik * spsik * sthetak) * dx + (cthetak * spsik + cpsik * sphik * sthetak) * dy - cphik * sthetak * dz;
    Ct(2, 20) = -(spsik * sthetak - cpsik * cthetak * sphik) * dx + (cpsik * sthetak + cthetak * sphik * spsik) * dy;


    // Row 3
    Ct(3, 3) = 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (-cos(psi_b) * sin(phi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + sin(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_k));
    Ct(3, 5) = -1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (cos(phi_b) * cos(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * sin(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)));
    Ct(3, 18) = -cos(theta_k) * 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k));
    Ct(3, 19) = -1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k));
    Ct(3, 20) = 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (cos(phi_b) * cos(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * sin(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)));
    
    // Row 4
    Ct(4, 3) = (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * (sin(psi_b) * sin(psi_k) * sin(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b)) * (-cos(phi_k) * cos(theta_k) * sin(phi_b) - cos(phi_b) * cos(psi_b) * sin(psi_k) * sin(theta_k) + cos(phi_b) * cos(psi_k) * sin(psi_b) * sin(theta_k) + cos(phi_b) * cos(psi_b) * cos(psi_k) * cos(theta_k) * sin(phi_k) + cos(phi_b) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(psi_k)) * 1.0 / pow(cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_b) * sin(theta_k) + sin(psi_b) * sin(psi_k) * sin(theta_b) * sin(theta_k) - cos(psi_b) * cos(theta_b) * sin(phi_b) * sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_b) * sin(phi_b) * sin(psi_b) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) * sin(theta_b) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(theta_b) + cos(psi_b) * cos(psi_k) * cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) + cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 4) = ((1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0) + 1.0) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 5) = ((((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k))) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k))) * 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 18) = -(((-cos(phi_k) * cos(psi_k) * cos(theta_k) * (cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) + cos(phi_k) * cos(theta_k) * sin(psi_k) * (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) + cos(phi_b) * cos(theta_k) * sin(phi_k) * sin(theta_b)) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b)) * (cos(phi_k) * cos(psi_k) * cos(theta_k) * (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) - cos(phi_k) * cos(theta_k) * sin(psi_k) * (cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) + cos(phi_b) * cos(theta_b) * cos(theta_k) * sin(phi_k))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 19) = -((sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * 1.0 / pow(cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_b) * sin(theta_k) + sin(psi_b) * sin(psi_k) * sin(theta_b) * sin(theta_k) - cos(psi_b) * cos(theta_b) * sin(phi_b) * sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_b) * sin(phi_b) * sin(psi_b) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) * sin(theta_b) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(theta_b) + cos(psi_b) * cos(psi_k) * cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) + cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 20) = -((((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k))) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k))) * 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    
    // Row 5
    Ct(5, 3) = (sin(psi_b) * sin(psi_k) * sin(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 5) = (((cos(phi_b) * cos(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k))) / (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) + sin(psi_b - psi_k) * cos(phi_b) * cos(phi_k) * 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 18) = -((sin(theta_k) - 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_k) * sin(phi_b) + cos(phi_b) * cos(psi_b) * cos(psi_k) * sin(phi_k) + cos(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k)) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 19) = ((sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) * (cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b))) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 20) = -(((cos(phi_b) * cos(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k))) / (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) + sin(psi_b - psi_k) * cos(phi_b) * cos(phi_k) * 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));

    return Ct;
  }


Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v) {

  // TODO
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;

  I6.setIdentity();

  return I6;
}

}  // namespace ekf_imu_vision
