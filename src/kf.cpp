
//
// src: kf.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   Kalman Filterの計算をするためのクラス
//

#include "kf_tracking/kf.h"
// #include "mmath/binarion.h"

using namespace std;

KalmanFilter::KalmanFilter()
{
}

Eigen::Matrix4d KalmanFilter::F(const double &dt)
{
	Eigen::Matrix4d F;

	F << 1.0, 0.0,  dt, 0.0,
	     0.0, 1.0, 0.0,  dt,
	     0.0, 0.0, 1.0, 0.0,
	     0.0, 0.0, 0.0, 1.0;

	return F;
}

Eigen::Matrix<double, 4, 2> KalmanFilter::G(const double &dt)
{
	Eigen::Matrix<double, 4, 2> G;

	G << dt * dt / 2,           0.0,
	             0.0,   dt * dt / 2,
	              dt,           0.0,
	             0.0,            dt;

	return G;
}

Eigen::Matrix2d KalmanFilter::Q()
{
	Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
	double sig_a = 0.05;

	Q = sig_a * Q;

	return Q;
}

Eigen::Matrix4d KalmanFilter::covGw(const double &dt)
{
	Eigen::Matrix4d Q;
	double sig_a = 0.05;

	Q << pow(dt, 4) / 4.0,              0.0,        0.0,        0.0,
	                  0.0, pow(dt, 4) / 4.0,        0.0,        0.0,
	     pow(dt, 3) / 2.0,              0.0, pow(dt, 2),        0.0,
	                  0.0, pow(dt, 3) / 2.0,        0.0, pow(dt, 2);

	Q = sig_a * Q;

	return Q;
}

Eigen::Matrix<double, 2, 4> KalmanFilter::H()
{
	Eigen::Matrix<double, 2, 4> H;

	H << 1.0, 0.0, 0.0, 0.0,
	     0.0, 1.0, 0.0, 0.0;

	return H;
}

