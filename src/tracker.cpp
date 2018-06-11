
//
// src: tracker.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   クラスタを管理するクラス(対応付けとか)
//

#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/impl/io.hpp>// for copyPointCloud
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include "kf_tracking/tracker.h"
#include "mmath/common.h"

using namespace std;

Tracker::Tracker()
	: sigma_p(0.05), sigma_r(0.1), SDTH(0.6), ELTH(0.5), frame_id("/map"),
	  isIncrease(false), isStatic(false)
{
	pub_track = n.advertise<sensor_msgs::PointCloud2>("/tracking_points", 1);
	pub_arrow = n.advertise<visualization_msgs::MarkerArray>("/velocity_arrows", 1);
	pub_ellipse = n.advertise<visualization_msgs::MarkerArray>("/error_ellipses", 1);
	pub_links = n.advertise<visualization_msgs::MarkerArray>("/link_lines", 1);

	link.header.stamp = ros::Time::now();
	link.header.frame_id = "/map";

	link.ns = "/cluster/links";
	link.id = 0;

	link.type = visualization_msgs::Marker::LINE_LIST;
	link.action = visualization_msgs::Marker::ADD;

	link.pose.orientation.w = 1.0;

	link.scale.x = 1.0;

	link.color.b = 1.0;
	link.color.r = 0.6;
	link.color.a = 1.0;

	link.lifetime = ros::Duration(1.0);
}

void Tracker::setIncrease()
{
	isIncrease = true;
}

void Tracker::setIncrease(const bool flag)
{
	isIncrease = flag;
}

void Tracker::setStatic()
{
	isStatic = true;
}

void Tracker::setStatic(const bool flag)
{
	isStatic = flag;
}

void Tracker::setSigma(const double &sig_p, const double &sig_r)
{
	sigma_p = sig_p;
	sigma_r = sig_r;
}

void Tracker::setThresholdSame(const double &dist)
{
	SDTH = dist;
}

void Tracker::setThresholdErase(const double &likelihood)
{
	ELTH = likelihood;
}

void Tracker::setFrameID(const string frame_name)
{
	frame_id = frame_name;
}

void Tracker::setPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	int lifetime;
	transform(pc_in, pc);
	associate(pc);
	update(pc);

	auto it = clusters.begin();
	while( it != clusters.end()){
		it->second.predict();
		if(it->second.likelihood() < ELTH ||
				(it->second.age() < 5 && 5 < it->second.consecutiveInvisibleCount()) ||
				15 < it->second.consecutiveInvisibleCount()){
			lifetime = it->second.getLifetime();
			if(lifetime < 1){
				clusters.erase(it++);
			}else{
				it->second.setLifetime(lifetime - 1);
				++it;
			}
		}else{
			it->second.setLifetime(10);
			++it;
		}
	}
}

template<class T_pc>
void Tracker::setPosition(const T_pc &pc)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*pc, *pc_xyz);
	
	setPosition(pc_xyz);
}
template void Tracker::setPosition<pcl::PointCloud<pcl::PointNormal>::Ptr>(const pcl::PointCloud<pcl::PointNormal>::Ptr&);
template void Tracker::setPosition<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr&);
template void Tracker::setPosition<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

void Tracker::setParams()
{
}

void Tracker::pubTrackingPoints()
{
	sensor_msgs::PointCloud2 pc2;
	pcl::PointCloud<pcl::PointNormal>::Ptr pc(new pcl::PointCloud<pcl::PointNormal>);

	for(auto it = clusters.begin(); it != clusters.end(); ++it){
		if(it->second.totalVisibleCount() > 10){
			it->second.getTrackingPoint(pc, it->first);
		}
	}
	if(isStatic){
		pc->header.frame_id = frame_id;
	}else{
		pc->header.frame_id = "/map";
	}
	// pc->header.frame_id = frame_id;
	pcl::toROSMsg(*pc, pc2);
	pc2.header.stamp = ros::Time::now();

	pub_track.publish(pc2);
}

void Tracker::pubVelocityArrows()
{
	visualization_msgs::MarkerArray arrows;

	for(auto it = clusters.begin(); it != clusters.end(); ++it){
		if(it->second.totalVisibleCount() > 10){
			it->second.getVelocityArrow(arrows, it->first);
		}
	}

	if(arrows.markers.size()){
		pub_arrow.publish(arrows);
	}
}

void Tracker::pubErrorEllipses()
{
	visualization_msgs::MarkerArray ellipses;

	for(auto it = clusters.begin(); it != clusters.end(); ++it){
		if(it->second.totalVisibleCount() > 10){
			it->second.getErrorEllipse(ellipses, it->first);
		}
	}

	if(ellipses.markers.size()){
		pub_ellipse.publish(ellipses);
	}
}

void Tracker::pubLinkLines()
{

	if(links.markers.size()){
		pub_links.publish(links);
	}
	links.markers.clear();
}

ostream& operator << (ostream &os, const Tracker &tracker)
{
	
	os << "clusters size : " << tracker.clusters.size() << endl;

	for(auto it = tracker.clusters.begin(); it != tracker.clusters.end(); ++it){
	   os << "  cluster[" << it->first << "]\n" 
	      << it->second << "\n";
	}

	return os;
}


///////////// private /////////////////

int Tracker::getID(const int& idx)
{
	int id = 0;
	bool found = false;

	for(auto it = clusters.begin(); it != clusters.end(); ++it, ++id){
		if(id == idx){
			id = it->first;
			found = true;
			break;
		}
	}

	if(!found) id = -1;

	return id;
}

int Tracker::getNewID()
{
	if(isIncrease){
		static int id_back = 0;
		return id_back++;
	}

	int id_new = 0;

	for(auto it = clusters.begin(); it != clusters.end() && it->first == id_new; ++it, ++id_new);

	return id_new;
}

int Tracker::getCost(const Cluster& cluster, const pcl::PointXYZ& p)
{
	Cluster vc(cluster); // virtual_cluster
	double pre_likelihood = vc.likelihood();
	double likelihood;

	vc.measurementUpdate(p);
	vc.predict();
	likelihood = vc.getLikelihood();

	double rate = (likelihood - pre_likelihood) / likelihood;
	double dist = cluster.getDist(p);

	// cout << "rate : " << rate << endl;
	return int(100.0*dist + 10.0*dist*rate);
	// return int(100.0*dist);
}

int Tracker::getCost(const Cluster& cluster)
{
	// virtual_cluster = cluster;
	// // double pre_likelihood = virtual_cluster.getLikelihood();
    //
	// virtual_cluster.predict();
    //
	// // return 100 * SDTH + virtual_cluster.getLikelihood();
	// // return 100 * SDTH + (virtual_cluster.getLikelihood() - pre_likelihood);
	return int(100.0 * SDTH);
}

int Tracker::getCost(const pcl::PointXYZ& p)
{
	// virtual_cluster = Cluster(p, sigma_p, sigma_r);
    //
	// virtual_cluster.predict();
    //
	// // return 100 * SDTH + virtual_cluster.getLikelihood();
	return int(100.0 * SDTH);
}

int Tracker::getCost()
{
	// virtual_cluster = Cluster();
    //
	// // virtual_cluster.predict();
    //
	// // return 100 * SDTH + virtual_cluster.getLikelihood();
	return int(100.0 * SDTH);
}

void Tracker::hungarianSolve(Eigen::MatrixXi& M)
{
	const int Inf = 1e6;

	int n = M.rows(), p, q;
	vector<int> fx(n, Inf), fy(n, 0);
	vector<int> x(n, -1), y(n, -1);

	for(int i = 0; i < n; ++i){
		for(int j = 0; j < n; ++j){
			fx[i] = min(fx[i], M(i, j));
		}
	}

	for(int i = 0; i < n; ){
		vector<int> t(n, -1), s(n+1, i);
		for(p = q = 0; p <= q && x[i] < 0; ++p){
			for(int k = s[p], j = 0; j < n && x[i] < 0; ++j){
				if (fx[k] + fy[j] == M(k, j) && t[j] < 0){
					s[++q] = y[j], t[j] = k;
					if(s[q] < 0){
						for(p = j; p >= 0; j = p){
							y[j] = k = t[j], p = x[k], x[k] = j;
						}
					}
				}
			}
		}
		if(x[i] < 0){
			int d = Inf;
			for(int k = 0; k <= q; ++k){
				for(int j = 0; j < n; ++j){
					if(t[j] < 0) d = max(d, fx[s[k]] + fy[j] - M(s[k], j));
				}
			}
			for(int j = 0; j < n; ++j) fy[j] += (t[j] < 0 ? 0 : d);
			for(int k = 0; k <= q; ++k) fx[s[k]] -= d;
		}else ++i;
	}

	neighbors.resize(n);
	for(int i=0; i<n; ++i){
		neighbors[i] = getID(y[i]);
	}
}

void Tracker::associate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pos)
{
	const int c_size = clusters.size();
	const int p_size = pos->points.size();
	const int m_size = c_size + p_size;

	Eigen::MatrixXi M(m_size, m_size);

	for(int i=0; i<m_size; ++i){
		for(int j=0; j<m_size; ++j){
			if(i < c_size && j < p_size){
				M(i, j) = getCost(clusters[getID(i)], pos->points[j]);
			}else if(i < c_size && !(j < p_size)){
				M(i, j) = getCost(clusters[getID(i)]);
			}else if(!(i < c_size) && j < p_size){
				M(i, j) = getCost(pos->points[i]);
			}else{ // !(i < c_size) && !(j < p_size)
				M(i, j) = getCost();
			}
		}
	}

	hungarianSolve(M);
}

void Tracker::update(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
	int c_id;

	for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
		c_id = neighbors[it - pc->points.begin()];
		auto itr = clusters.find(c_id);
		if(itr != clusters.end()){
			link.header.stamp = ros::Time::now();
			link.id = itr->first;
			link.color.r = 0.0 + 0.5 * (itr->first % 3);
			link.color.g = 0.4 + 0.1 * (itr->first % 7);
			link.color.b = 0.2 + 0.2 * (itr->first % 5);
			itr->second.getLinkLines(link, *it);
			links.markers.push_back(link);
			link.points.clear();
			itr->second.measurementUpdate(*it);
		}else{
			Cluster cluster(*it, sigma_p, sigma_r);
			if(isStatic){
				cluster.setFrameID(frame_id);
			}
			clusters[getNewID()] = cluster;
		}
	}
}

void Tracker::transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_out)
{
	*pc_out = *pc_in;

	if(isStatic || frame_id == "/map" || frame_id == "map") return;

	ros::Time past_time;
	pcl_conversions::fromPCL(pc_in->header.stamp, past_time);

	// pcl::PointXYZ p;
	tf::StampedTransform transform;
	geometry_msgs::PointStamped ps_in, ps_out;

	ps_in.header.frame_id = frame_id;
	

	// for文回さずにpclのtransformで変換したほうがいいでしょ
	try{
		// listener_.waitForTransform("/map", frame_id, past_time, ros::Duration(0.05));
		// listener_.lookupTransform("/map", frame_id, past_time, transform);
		for(auto it = pc_out->points.begin(); it != pc_out->points.end(); ++it){
			ps_in.point.x = it->x;
			ps_in.point.y = it->y;
			ps_in.point.z = it->z;
			listener_.waitForTransform("/map", frame_id, past_time, ros::Duration(0.05));
			listener_.transformPoint("/map", ps_in, ps_out);
			it->x = ps_out.point.x;
			it->y = ps_out.point.y;
			it->z = ps_out.point.z;
			// p.x = it->x + transform.getOrigin().x();
			// p.y = it->x + transform.getOrigin().y();
			// p.z = 0.0;
			// pc_out->points.push_back(p);
		}
	}catch(tf::TransformException ex){
		ROS_ERROR("%s\n", ex.what());
		ros::Duration(1.0).sleep();
	}
}

