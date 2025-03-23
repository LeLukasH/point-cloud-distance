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
class MainWindow;

class Compute {
public:
    explicit Compute(MainWindow* mainWindow);

    QString computeHausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    QString computeChamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    QString computeEarthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    QString computeJensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    vector<float> getDistances(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);

private:
    MainWindow* mainWindow; // Store a pointer to MainWindow

    static double klDivergence(const vector<double> &P, const vector<double> &Q);
    static vector<double> normalizeDistribution(const vector<float> &distances);
};

#endif // COMPUTE_H
