
//
// include: cluster.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   tracking対象のそれぞれのクラスタclass
//   kfをもちいて計算(二次元で推定)
//   位置のみを使用
//

#ifndef CLUSTER_H
#define CLUSTER_H

#include <ros/ros.h>// ostream等
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include "mmath/differential.h"
#include "mmath/pca.h"
#include "kf_tracking/kf.h"

class Cluster
{
	//feature <-- 今後増やしていくけるように.. 曲率とかサイズとか色とか(Trackerの方で判断)
	// int id;
	double likelihood_;

	KalmanFilter kf;
	Eigen::Vector4d x;	    // システム(系)の状態推定値(x, y, vx, vy) 4x1
	// Eigen::Vector2d G;  	// 時間遷移モデル(x'', y'') 2x1
	Eigen::Matrix4d P;  	// 誤差の共分散行列(推定値の精度) 4x4
	// Eigen::Matrix2d Q;  	// 共分散行列(時間変化) 2x2
	Eigen::Matrix2d R;		// 共分散行列(観測の信頼度) 2x2
	Eigen::Vector2d obs;	// 観測(x, y) 2x1

	Eigen::Vector2d y;	    // 推定値から求まる系の状態(v, θ) 2x1

	int lifetime;
	int age_; //トラックが最初に検出されてからのフレーム数
	int totalVisibleCount_; //トラックが検出されたフレームの合計数
	int consecutiveInvisibleCount_; //トラックが連続して検出されなかったフレームの数
	std::string frame_id;

	ros::Time current_time, last_time; // Trackerでdt出したほうが高速だけど

	// Differential vx;
	// Differential vy;
	PrincipalComponentAnalysis pca;

	template<class T_p>
	void initialize(const ros::Time&, const T_p&, const double&, const double&);
	void setY();

	public:
	Cluster();
	Cluster(const Cluster&);
	Cluster(const ros::Time&, const pcl::PointXYZ&, const double&, const double&);
	void measurementUpdate(const pcl::PointXYZ&);
	void predict(const ros::Time&);
	void setParams();
	void setFrameID(const std::string&);
	void setLifetime(const int);
	void setSigma(double);
	void setLikelihood(const double&);
	int age() const;
	int totalVisibleCount() const;
	int consecutiveInvisibleCount() const;
	int getLifetime();
	double getDist(const geometry_msgs::Point&) const;
	double getDist(const pcl::PointXYZ&) const;
	double getDist2ObsLast(const pcl::PointXYZ&) const;
	void getPoint(pcl::PointXYZ&);
	void getPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	double likelihood();
	double getLikelihood();
	void getTrackingPoint(pcl::PointCloud<pcl::PointNormal>::Ptr&, const int);
	void getVelocityArrow(visualization_msgs::MarkerArray&, const int);
	void getErrorEllipse(visualization_msgs::MarkerArray&, const int);
	void getLinkLines(visualization_msgs::Marker&, const pcl::PointXYZ&);

	friend std::ostream& operator << (std::ostream&, const Cluster&);
};

#endif

