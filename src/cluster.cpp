
//
// src: cluster.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   tracking対象のそれぞれのクラスタclass
//

#include <tf/transform_broadcaster.h>
#include <Eigen/Dense> // for EigenSolver
#include "mmath/common/angles.h"
#include "mmath/binarion.h"
#include "kf_tracking/cluster.h"

using namespace std;

Cluster::Cluster()
	: likelihood_(1.0), lifetime(10), age_(0), totalVisibleCount_(0), consecutiveInvisibleCount_(0),
	  frame_id("/map")
	  //, vx(x(0)), vy(x(1))
{
	geometry_msgs::Point p;
	p.x = p.y = 0.0;
	initialize(ros::Time(0.0), p, 10.0, 0.01);
}

Cluster::Cluster(const Cluster& cluster)
{
	likelihood_ = cluster.likelihood_;
	x = cluster.x;
	P = cluster.P;
	R = cluster.R;
	obs = cluster.obs;
	y = cluster.y;
	lifetime = cluster.lifetime;
	age_ = cluster.age_;
	totalVisibleCount_ = cluster.totalVisibleCount_;
	consecutiveInvisibleCount_ = cluster.consecutiveInvisibleCount_;
	current_time = cluster.current_time;
	last_time = cluster.last_time;
	frame_id = cluster.frame_id;
	// pca は共有できない
}

Cluster::Cluster(const ros::Time& t, const pcl::PointXYZ &p, const double &sig_p, const double &sig_r)
	: likelihood_(1.0), lifetime(10), age_(0), totalVisibleCount_(0), consecutiveInvisibleCount_(0),
	  frame_id("/map")
	  //, vx(x(0)), vy(x(1))
{
	initialize(t, p, sig_p, sig_r);
}

void Cluster::measurementUpdate(const pcl::PointXYZ &p)
{
	const int N = 10; //pcaで貯める点の数
	pca.setPoints2d(p, N);

	obs << p.x, p.y;

	Eigen::Matrix4d I  = Eigen::Matrix4d::Identity();
	Eigen::Matrix<double, 2, 4> H = kf.H(); //観測モデルのヤコビアン
	Eigen::Vector2d e = obs - H * x; //観測残差、innovation
	Eigen::Matrix2d S = H * P * H.transpose() + R; // 観測残差の共分散
	Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse(); // 最適 カルマンゲイン

	x = x + K * e; // 更新された状態の推定値
	P = (I - K * H) * P; // 更新された誤差の共分散

	// cout << "K :\n" << K << endl;

	totalVisibleCount_++;
	consecutiveInvisibleCount_ = 0;

	// cout << "updated :\n" << x << endl;
}

void Cluster::predict(const ros::Time& t)
{
	// current_time = ros::Time::now();
	current_time = t;
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	if(0.1 < dt) return;

	Eigen::Matrix4d F = kf.F(dt); // 動作モデルのヤコビアン
	// Eigen::Matrix<double, 4, 2> G = kf.G(dt); // ノイズ(ax, ay)に関するモデル
	// Eigen::Matrix4d Q = kf.Q(dt); // ノイズモデルの共分散行列
	Eigen::Matrix4d Q = kf.covGw(dt); // G * Q * G.transpose();
	x = F * x; // 時間発展動作モデル
	P = F * P * F.transpose() + Q;
	setY();

	age_++;
	consecutiveInvisibleCount_++;

	// cout << "predicted : \n" << x << endl;
}

void Cluster::setParams()
{
}

void Cluster::setFrameID(const string& frame_name)
{
	frame_id = frame_name;
}

void Cluster::setLifetime(const int life)
{
	lifetime = life;
}

void Cluster::setLikelihood(const double &sigma)
{
	likelihood_ = sigma;
}

int Cluster::age() const
{
	return age_;
}

int Cluster::totalVisibleCount() const
{
	return totalVisibleCount_;
}

int Cluster::consecutiveInvisibleCount() const
{
	return consecutiveInvisibleCount_;
}

int Cluster::getLifetime()
{
	return lifetime;
}

double Cluster::getDist(const geometry_msgs::Point &point) const
{
	pcl::PointXYZ p;

	p.x = point.x;
	p.y = point.y;
	p.z = point.z;

	return getDist(p);
}

double Cluster::getDist(const pcl::PointXYZ &p) const
{
	return sqrt(pow(p.x - x(0), 2) + pow(p.y - x(1), 2));
}

double Cluster::getDist2ObsLast(const pcl::PointXYZ &p) const
{
	return sqrt(pow(p.x - obs(0), 2) + pow(p.y - obs(1), 2));
}

void Cluster::getPoint(pcl::PointXYZ &p)
{
	p.x = x(0);
	p.y = x(1);
	p.z = 0.0;
}

void Cluster::getPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{
	pcl::PointXYZ p;

	p.x = x(0);
	p.y = x(1);
	p.z = 0.0;

	pc->points.push_back(p);
}

double Cluster::likelihood()
{
	return likelihood_;
}

double Cluster::getLikelihood()
{
	double a, b;
	Eigen::Matrix2d m = P.block<2, 2>(0, 0);
	Eigen::EigenSolver<Eigen::Matrix2d> es(m);
	if(!es.info()){ // == "Success"
		Eigen::Vector2d values = es.eigenvalues().real();
		double lambda1, lambda2;
		double kai2 = 9.21034; // χ² (chi-square) distribution 99% (95%:5.99146)
		if(values(0) < values(1)){
			lambda1 = values(1);
			lambda2 = values(0);
		}else{
			lambda1 = values(0);
			lambda2 = values(1);
		}
		a = sqrt(kai2 * lambda1);
		b = sqrt(kai2 * lambda2);
		if(a*b < 1e-5){
			likelihood_ = 1e6;
		}else{
			likelihood_ = 10 / (a * b);
		}
	}else{
		cerr << "Eigen solver error info : " << es.info() << endl;
	}
	return likelihood_;
}

void Cluster::getTrackingPoint(pcl::PointCloud<pcl::PointNormal>::Ptr &pc, const int id)
{
	pcl::PointNormal pn;

	pn.x = x(0);
	pn.y = x(1);
	pn.z = 0.0;
	pn.curvature = id;

	pc->points.push_back(pn);
}

void Cluster::getVelocityArrow(visualization_msgs::MarkerArray &markers, const int id)
{
	visualization_msgs::Marker arrow;

	// marker.header.frame_id = "/map";
	arrow.header.stamp = current_time;
	arrow.header.frame_id = frame_id;

	arrow.ns = "/cluster/arrow";
	arrow.id = id;

	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.action = visualization_msgs::Marker::ADD;
	// arrow.action = visualization_msgs::Marker::DELETE;
	// arrow.action = visualization_msgs::Marker::DELETEALL;

	arrow.pose.position.x = x(0);
	arrow.pose.position.y = x(1);
	arrow.pose.position.z = 0.0;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(y(1));
	arrow.pose.orientation = odom_quat;

	// arrow.scale.x = 1.0; // length
	arrow.scale.x = y(0); // length
	arrow.scale.y = 0.1; // width
	arrow.scale.z = 0.1; // height

	arrow.color.r = 1.0f;
	arrow.color.g = 0.0f;
	arrow.color.b = 0.9f;
	arrow.color.a = 0.6;

	arrow.lifetime = ros::Duration(1.0);

	markers.markers.push_back(arrow);
}

void Cluster::getErrorEllipse(visualization_msgs::MarkerArray &markers, const int id)
{
	visualization_msgs::Marker ellipse;
	ellipse.header.stamp = current_time;
	ellipse.header.frame_id = frame_id;

	ellipse.ns = "/cluster/ellipse";
	ellipse.id = id;

	ellipse.type = visualization_msgs::Marker::CYLINDER;
	ellipse.action = visualization_msgs::Marker::ADD;

	double a, b;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
	Eigen::Matrix2d m = P.block<2, 2>(0, 0);
	Eigen::EigenSolver<Eigen::Matrix2d> es(m);
	if(!es.info()){ // == "Success"
		Eigen::Vector2d values = es.eigenvalues().real();
		Eigen::Matrix2d vectors = es.eigenvectors().real();
		Eigen::Vector2d vec;
		double lambda1, lambda2, theta;
		theta = 0.0;
		double kai2 = 9.21034; // χ² (chi-square) distribution 99% (95%:5.99146)
		if(values(0) < values(1)){
			lambda1 = values(1);
			lambda2 = values(0);
			vec = vectors.col(1);
		}else{
			lambda1 = values(0);
			lambda2 = values(1);
			vec = vectors.col(0);
		}
		theta = atan2(vec(1), vec(0));
		odom_quat = tf::createQuaternionMsgFromYaw(theta);
		a = sqrt(kai2 * lambda1);
		b = sqrt(kai2 * lambda2);
		if(a*b < 1e-5){
			likelihood_ = 1e6;
		}else{
			likelihood_ = 10 / (a * b);
		}
		ellipse.pose.position.x = x(0);
		ellipse.pose.position.y = x(1);
		ellipse.pose.position.z = 0.0;
		ellipse.pose.orientation = odom_quat;

		ellipse.scale.x = a;
		ellipse.scale.y = b;
		ellipse.scale.z = 0.01;

		ellipse.color.r = 0.0f;
		ellipse.color.g = 1.0f;
		ellipse.color.b = 1.0f;
		ellipse.color.a = 0.3;

		ellipse.lifetime = ros::Duration(1.0);

		markers.markers.push_back(ellipse);
	}else{
		cerr << "Eigen solver error info : " << es.info() << endl;
	}
}

void Cluster::getLinkLines(visualization_msgs::Marker& link, const pcl::PointXYZ& tgt)
{
	geometry_msgs::Point p;

	p.x = obs(0);
	p.y = obs(1);
	// p.x = x(0);
	// p.y = x(1);
	link.points.push_back(p);

	p.x = tgt.x + 0.5;
	p.y = tgt.y + 0.5;
	link.points.push_back(p);
}

ostream& operator << (ostream &os, const Cluster &cluster)
{
	os << "    position : (" << cluster.x(0) << ", " << cluster.x(1) << ")\n"
	   << "    likelihood : " << cluster.likelihood_ << ", " 
	   << "    age : " << cluster.age_ << ", " 
	   << "    totalVisibleCount : " << cluster.totalVisibleCount_ << ", " 
	   << "    consecutiveInvisibleCount : " << cluster.consecutiveInvisibleCount_ << endl;

	return os;
}


///////////////// private //////////////////////
template<class T_p>
void Cluster::initialize(const ros::Time& t, const T_p& p, const double &sig_p, const double &sig_r)
{
	current_time = t;
	last_time = current_time;

	x << p.x, p.y, 0.0, 0.0;

	obs << p.x, p.y;

	P << sig_p,     0,     0,     0,
	         0, sig_p,     0,     0,
	         0,     0, 100.0,     0,
	         0,     0,     0, 100.0;

	R << sig_r,      0,
	         0,  sig_r;
}

void Cluster::setY()
{
	// static Differential v(x(0), x(1)); // インスタンスごとではなくclassごとに共有されてしまう
	
	// y(0) = v.get(x(0), x(1));
	// v_x = vx.get(x(0));
	// v_y = vy.get(x(1));
	y(0) = sqrt(pow(x(2), 2) + pow(x(3), 2));
	// y(0) = sqrt(pow(v_x, 2) + pow(v_y, 2));

	if(y(0) > 0.09){ // (1km/h : 0.2777m/s)
		Eigen::Vector3d vec = pca.vector(0); // 移動の向き(vec --> θ) 
		// y(1) = atan2(v_y, v_x);
		y(1) = atan2(vec(1), vec(0));
		double div = Binarion::deviation(y(1), pca.direction());

		if(div > M_PI / 2){
			y(1) += M_PI;
		}else if(div < - M_PI / 2){
			y(1) -= M_PI;
		}
	}
	
}

