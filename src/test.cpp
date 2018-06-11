
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
// #include "visualization_tools/bounding_box.h"
#include "kf_tracking/tracker.h"

using namespace std;

class HumanTracker
{
	ros::NodeHandle n;
	ros::Subscriber sub;
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

	Tracker tracker;

	public:
	HumanTracker()
		: pc(new pcl::PointCloud<pcl::PointXYZ>)
	{
		sub = n.subscribe<sensor_msgs::PointCloud2>("/human_recognition/positive_position", 1,
				&HumanTracker::humanCallback, this);

		// tracker.setStatic();
		// tracker.setIncrease();
		tracker.setThresholdSame(0.8);
		tracker.setThresholdErase(0.15);
		tracker.setSigma(100.0, 0.01); // P, R
		tracker.setFrameID("/velodyne");
	}

	void humanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		pcl::fromROSMsg(*msg, *pc);
		tracking();
	}

	void tracking();
};

void HumanTracker::tracking()
{
	tracker.setPosition(pc);

	cout << "\033[2J"; //画面クリア
	printf("\033[%d;%dH", 1, 1); //カーソル位置を、高さ1行目、横1行目に移動
	cout << tracker << endl;
	tracker.pubTrackingPoints();
	tracker.pubVelocityArrows();
	tracker.pubErrorEllipses();
	// tracker.pubLinkLines();
	// cout << "pc size : " << pc->points.size() << endl;
	// cout << "================\n" << endl;
	// cout << "================" << endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_tracking_kf");
	cout << "kf tracking" << endl;

	HumanTracker ht;

	ros::spin();

	return 0;
}

