#pragma once
#include "peac/AHCPlaneFitter.hpp"
#include "peac/AHCUtils.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
using namespace std;

template<class PointT>
struct OrganizedImage3D {
	const pcl::PointCloud<PointT>& cloud;
	//note: ahc::PlaneFitter assumes mm as unit!!!
	const double unitScaleFactor;

	OrganizedImage3D(const pcl::PointCloud<PointT>& c) : cloud(c), unitScaleFactor(1) {}
	OrganizedImage3D(const OrganizedImage3D& other) : cloud(other.cloud), unitScaleFactor(other.unitScaleFactor) {}

	inline int width() const { return cloud.width; }
	inline int height() const { return cloud.height; }
	inline bool get(const int row, const int col, double& x, double& y, double& z) const {
		const PointT& pt=cloud.at(col,row);
		x=pt.x*unitScaleFactor; y=pt.y*unitScaleFactor; z=pt.z*unitScaleFactor; //TODO: will this slowdown the speed?
		return std::isnan(z)==0; //return false if current depth is NaN
	}
};

typedef OrganizedImage3D<pcl::PointXYZ> ImageXYZ;
typedef ahc::PlaneFitter< ImageXYZ > PlaneFitter;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;
class plane_detection
{
private:
    PlaneFitter pf;
    bool showWindow;
    bool save_flag;
    double unitScaleFactor;
    std::string outputDir;
	
    int process_index = 0;
public:
	vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > planes_info; // normal and center
	vector<cv::Mat> planes;
	cv::Mat result;
    plane_detection();

    void detect(pcl::PointCloud<pcl::PointXYZ> & pc);
    ~plane_detection();
};