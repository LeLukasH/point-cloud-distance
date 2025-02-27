#ifndef COMPUTE_H
#define COMPUTE_H

#include <QString>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using std::vector;

class Compute {
public:
    static QString computeHausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    static QString computeChamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    static QString computeEarthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    static QString computeJensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    static vector<float> getDistances(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);

private:
    static double klDivergence(const vector<double> &P, const vector<double> &Q);
    static vector<double> normalizeDistribution(const vector<float> &distances);
};

#endif // COMPUTE_H
