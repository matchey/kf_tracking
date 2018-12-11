
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
	HumanTracker(ros::NodeHandle priv_nh)
		: pc(new pcl::PointCloud<pcl::PointXYZ>)
	{
		string topic_name;

		priv_nh.param<string>("sub_topic_name", topic_name, "/cluster/human/position");

		sub = n.subscribe<sensor_msgs::PointCloud2>(topic_name, 1,
													&HumanTracker::humanCallback, this);

		bool isIncrese;
		bool isStatic;
		string frame_id;

		priv_nh.param<bool>("flagIncrese", isIncrese, false);
		priv_nh.param<bool>("flagStatic", isStatic, false);
		priv_nh.param<string>("frameID", frame_id, "/velodyne");

		tracker.setStatic(isStatic);
		if(isIncrese) tracker.setIncrease();
		tracker.setThresholdSame(0.8);
		tracker.setThresholdErase(0.15);
		tracker.setSigma(100.0, 0.01); // P, R
		tracker.setFrameID(frame_id);
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

	ros::NodeHandle priv_nh("~");

	HumanTracker ht(priv_nh);

	ros::spin();

	return 0;
}

