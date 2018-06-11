
//
// include: kf.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   Kalman Filterの計算をするためのクラス
//   2次元の観測(x, y)から状態(x, y, vx, vy)を推定
//   制御入力は無し
//

#ifndef KF_H
#define KF_H

#include <Eigen/Core>

class KalmanFilter
{
	public:
	KalmanFilter();
	Eigen::Matrix4d F(const double&); // 時間発展動作モデル
	Eigen::Matrix<double, 4, 2> G(const double&); // ノイズw(=(ax, ay))に関するモデル
	Eigen::Matrix2d Q(); // ノイズモデルの共分散行列
	Eigen::Matrix4d covGw(const double&); // Gw共分散行列
	Eigen::Matrix<double, 2, 4> H(); // 観測モデル
};

#endif

