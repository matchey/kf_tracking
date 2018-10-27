
//
// include: tracker.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   クラスタを管理するクラス(対応付けとか)
//   それぞれのクラスタの位置はpcl::PointCloudでセットする(他のが都合よければ言ってください)
//   setPotion()を呼び出す周期でclusterを更新
//

#ifndef TRACKER_H
#define TRACKER_H

// #include <unordered_map>
#include <tf/transform_listener.h>
#include "kf_tracking/cluster.h"

class Tracker
{
	ros::NodeHandle n;
	ros::Publisher pub_track;
	ros::Publisher pub_arrow;
	ros::Publisher pub_ellipse;
	ros::Publisher pub_links;

	visualization_msgs::Marker link;
	visualization_msgs::MarkerArray links;

	tf::TransformListener listener_;

	std::map<int, Cluster> clusters; // 走査遅いからmap使うのよくない?
	Cluster virtual_cluster;
	double sigma_p; // conv of P(init)
	double sigma_r; // conv of R(obs)
	double SDTH; // same dist threshold
	double ELTH; // erase likelihood threshold
	std::string frame_id;
	std::vector<int> neighbors;
	bool isIncrease;
	bool isStatic;
	ros::Time current_time;

	int getID(const int&);
	int getNewID();
	int getCost(const Cluster&, const pcl::PointXYZ&);
	int getCost(const Cluster&);
	int getCost(const pcl::PointXYZ&);
	int getCost();
	void hungarianSolve(Eigen::MatrixXi&);
	void associate(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	void update(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr&, pcl::PointCloud<pcl::PointXYZ>::Ptr&);

	public:
	Tracker();
	void setIncrease();
	void setIncrease(const bool);
	void setStatic();
	void setStatic(const bool);
	void setSigma(const double&, const double&); //init P, R
	void setThresholdSame(const double&);
	void setThresholdErase(const double&);
	void setFrameID(const std::string);
	void setPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	template<class T_pc>
	void setPosition(const T_pc&);
	void setParams();
	void pubTrackingPoints();
	void pubVelocityArrows();
	void pubErrorEllipses();
	void pubLinkLines();

	friend std::ostream& operator << (std::ostream&, const Tracker&);
};

#endif

