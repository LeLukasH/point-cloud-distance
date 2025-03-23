#include "compute.h"
#include <cmath>
#include <algorithm>

#include "mainwindow.h"

Compute::Compute(MainWindow* mainWindow) : mainWindow(mainWindow) {}

// Function to calculate distances between points
vector<float> Compute::getDistances(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud(cloud_b);
    vector<float> distances(cloud_a->points.size());

    float exponent = mainWindow->getExponent();

    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        vector<float> sqr_distances(1); // To store squared distance of the nearest point

        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        distances[i] = std::pow(std::sqrt(sqr_distances[0]), exponent);
    }
    return distances;
}

// Function to compute Hausdorff Distance
QString Compute::computeHausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double max_dist = *std::max_element(distances.begin(), distances.end());

    return QString(
               "Hausdorff Distance computed\n"
               "(value is the longest shortest distance)\n"
               "\n"
               "value : %1\n")
        .arg(max_dist);
}

// Function to compute Chamfer Distance
QString Compute::computeChamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double sum = 0;
    for (size_t i = 0; i < distances.size(); i++) {
        sum += distances[i] * distances[i];
    }

    return QString(
               "Chamfer Distance computed\n"
               "(average squared distance)\n"
               "\n"
               "value : %1\n")
        .arg(sum / cloud_a->points.size());
}

// Function to compute Earth Movers Distance
QString Compute::computeEarthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    cv::Mat signature1(cloud_a->size(), 4, CV_32F); // [weight, x, y, z]
    cv::Mat signature2(cloud_b->size(), 4, CV_32F); // [weight, x, y, z]

    for (size_t i = 0; i < cloud_a->size(); ++i) {
        signature1.at<float>(i, 0) = 1.0f; // Uniform weight
        signature1.at<float>(i, 1) = cloud_a->points[i].x;
        signature1.at<float>(i, 2) = cloud_a->points[i].y;
        signature1.at<float>(i, 3) = cloud_a->points[i].z;
    }

    for (size_t i = 0; i < cloud_b->size(); ++i) {
        signature2.at<float>(i, 0) = 1.0f; // Uniform weight
        signature2.at<float>(i, 1) = cloud_b->points[i].x;
        signature2.at<float>(i, 2) = cloud_b->points[i].y;
        signature2.at<float>(i, 3) = cloud_b->points[i].z;
    }

    signature1.col(0) /= cv::sum(signature1.col(0))[0];
    signature2.col(0) /= cv::sum(signature2.col(0))[0];

    float emd = cv::EMD(signature1, signature2, cv::DIST_L2);

    return QString(
               "Earth Mover's Distance computed\n"
               "\n"
               "value : %1\n")
        .arg(static_cast<double>(emd));
}

// Function to calculate Kullback-Leibler divergence
double Compute::klDivergence(const vector<double> &P, const vector<double> &Q) {
    double kl_divergence = 0.0;

    for (size_t i = 0; i < P.size(); ++i) {
        if (P[i] != 0 && Q[i] != 0) {
            kl_divergence += P[i] * log(P[i] / Q[i]);
        }
    }

    return kl_divergence;
}

// Function to normalize distances into a probability distribution
vector<double> Compute::normalizeDistribution(const vector<float> &distances) {
    vector<double> distribution(distances.size());
    double sum = 0.0;

    for (double dist : distances) {
        sum += dist;
    }

    if (sum != 0) {
        for (size_t i = 0; i < distances.size(); ++i) {
            distribution[i] = static_cast<double>(distances[i]) / sum;
        }
    }

    return distribution;
}

// Function to compute Jensen-Shannon Divergence
QString Compute::computeJensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    if (cloud_a->size() != cloud_b->size()) return QString("Point clouds do not have the same number of points.");

    vector<float> distances_a = getDistances(cloud_a, cloud_b);
    vector<float> distances_b = getDistances(cloud_b, cloud_a);

    vector<double> P = normalizeDistribution(distances_a);
    vector<double> Q = normalizeDistribution(distances_b);

    vector<double> M(P.size());

    for (size_t i = 0; i < P.size(); ++i) {
        M[i] = 0.5 * (P[i] + Q[i]);
    }

    double jsd = 0.5 * klDivergence(P, M) + 0.5 * klDivergence(Q, M);

    return QString(
               "Jensen-Shannon Divergence computed\n"
               "(0 indicates identical distributions and 1 indicates maximum dissimilarity)\n"
               "\n"
               "value : %1\n")
        .arg(jsd);
}
