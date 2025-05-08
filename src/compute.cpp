#include "compute.h"
#include <cmath>
#include <algorithm>
#include "mainwindow.h"

Compute::Compute(MainWindow* mainWindow) : mainWindow(mainWindow) {}

/**
 * @brief Calculates the distances between two point clouds.
 *
 * This function computes the distance from each point in cloud_a to its closest
 * point in cloud_b. The distance is raised to the power of an exponent obtained
 * from the MainWindow.
 *
 * @param cloud_a A pointer to the first point cloud.
 * @param cloud_b A pointer to the second point cloud.
 * @return A vector of distances between corresponding points in cloud_a and cloud_b.
 */
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

/**
 * @brief Computes the Hausdorff Distance between two point clouds.
 *
 * The Hausdorff distance is the maximum of the minimum distances between points
 * in the two point clouds. It is used to measure the dissimilarity between two sets
 * of points.
 *
 * @param cloud_a A pointer to the first point cloud.
 * @param cloud_b A pointer to the second point cloud.
 * @return A string containing the calculated Hausdorff distance.
 */
QString Compute::computeHausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double max_dist = *std::max_element(distances.begin(), distances.end());

    return QString("Hausdorff Distance: %1\n").arg(max_dist);
}

/**
 * @brief Computes the Chamfer Distance between two point clouds.
 *
 * The Chamfer distance is the average of the squared distances between each point
 * in one cloud and its nearest neighbor in the other cloud.
 *
 * @param cloud_a A pointer to the first point cloud.
 * @param cloud_b A pointer to the second point cloud.
 * @return A string containing the computed Chamfer distance.
 */
QString Compute::computeChamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double sum = 0;
    for (size_t i = 0; i < distances.size(); i++) {
        sum += distances[i] * distances[i];
    }

    return QString("Chamfer Distance: %1\n").arg(sum / cloud_a->points.size());
}

/**
 * @brief Computes the Earth Movers Distance between two point clouds.
 *
 * The Earth Movers Distance (EMD) measures the effort required to transform one point
 * cloud into another by moving the "mass" (points). This is calculated using a transport
 * problem approach with the L2 distance metric.
 *
 * @param cloud_a A pointer to the first point cloud.
 * @param cloud_b A pointer to the second point cloud.
 * @return A string containing the computed Earth Movers Distance.
 */
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

    return QString("Earth Mover's Distance: %1\n").arg(static_cast<double>(emd));
}


/*
 * @brief Calculates the Kullback-Leibler divergence between two probability distributions.
 *
 * The Kullback-Leibler divergence is a measure of how one probability distribution diverges
 * from a second, expected probability distribution. It is asymmetric.
 *
 * @param P The first probability distribution.
 * @param Q The second probability distribution.
 * @return The Kullback-Leibler divergence between P and Q.
 */
/*
double Compute::klDivergence(const vector<double> &P, const vector<double> &Q) {
    double kl_divergence = 0.0;

    for (size_t i = 0; i < P.size(); ++i) {
        if (P[i] != 0 && Q[i] != 0) {
            kl_divergence += P[i] * log(P[i] / Q[i]);
        }
    }

    return kl_divergence;
}

/**
 * @brief Normalizes the distances into a probability distribution.
 *
 * This function converts the distance values into a normalized probability distribution
 * where the sum of all distances equals 1. It is used for distribution-based measures.
 *
 * @param distances A vector containing the distances to be normalized.
 * @return A vector representing the normalized probability distribution.
 */
/*
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

/**
 * @brief Computes the Jensen-Shannon Divergence between two point clouds.
 *
 * The Jensen-Shannon Divergence is a symmetrized and smoothed version of the Kullback-Leibler
 * divergence, often used as a measure of similarity between two probability distributions.
 *
 * @param cloud_a A pointer to the first point cloud.
 * @param cloud_b A pointer to the second point cloud.
 * @return A string containing the calculated Jensen-Shannon Divergence.
 */
/*
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
               "Jensen-Shannon Divergence:  %1\n"
               "(0 indicates identical distributions and 1 indicates maximum dissimilarity)\n")
        .arg(jsd);
}
*/

/**
 * @brief Computes a custom measure based on the number of identical points between two point clouds.
 *
 * This custom measure counts the number of identical points (i.e., points with a distance of 0)
 * between the two point clouds and returns this count along with the percentage.
 *
 * @param cloud_a A pointer to the first point cloud.
 * @param cloud_b A pointer to the second point cloud.
 * @return A string containing the number of identical points and their percentage.
 */
QString Compute::computeCustomMeasure1(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    vector<float> distances = getDistances(cloud_a, cloud_b);

    float count = 0.0;
    for (auto d : distances) {
        if (d == 0) {
            count++;
        }
    }

    float result = count / distances.size();
    float scaled_result = result * 100.0f;
    return QString("Number of identical points:  %1 (%2%)\n")
        .arg(count)
        .arg(QString::number(scaled_result, 'f', 2));
}

/**
 * @brief Computes a custom measure based on distances between two point clouds.
 *
 * This function calculates the median distance between points in two point clouds, `cloud_a` and `cloud_b`.
 * It then computes the sum of distances that are greater than twice the median distance (outliers) and returns
 * a string with the total outlier distance.
 *
 * @param[in] cloud_a First point cloud.
 * @param[in] cloud_b Second point cloud.
 * @return QString String representation of the total outlier distance.
 */
QString Compute::computeCustomMeasure2(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    // Get the list of distances between points in both clouds
    std::vector<float> distances = getDistances(cloud_a, cloud_b);

    // Set the threshold value for outliers
    float threshold = 2.0f;

    // Sort the distances to compute median
    std::sort(distances.begin(), distances.end());

    float median;
    size_t n = distances.size();
    if (n == 0) {
        return "No distances available.";
    } else if (n % 2 == 0) {
        median = (distances[n / 2 - 1] + distances[n / 2]) / 2.0f;
    } else {
        median = distances[n / 2];
    }

    float total = 0.0f;
    // Sum the distances that are greater than twice the median distance
    for (auto d : distances) {
        if (d >= median * threshold) {
            total += d;
        }
    }

    // Return the total outlier distance
    return QString(
               "Outliers total distance:  %1\n"
               "(Value is the sum of distances greater than 2 * median_distance.)\n")
        .arg(total);
}


/**
 * @brief Data structure for holding distance and indices between point clouds.
 *
 * This structure holds the distance between two points and their respective indices
 * in each of the point clouds. It also defines a comparison operator to allow sorting
 * by distance.
 */
struct Data {
    float distance; ///< Distance between the points.
    int indexA;    ///< Index of the point in the first point cloud.
    int indexB;    ///< Index of the point in the second point cloud.

    /**
     * @brief Comparison operator for sorting based on distance.
     *
     * This operator is used to sort `Data` objects in ascending order of distance.
     *
     * @param[in] other Another `Data` object to compare with.
     * @return true if the current object has a smaller distance than the other.
     * @return false otherwise.
     */
    bool operator<(const Data& other) const {
        return distance < other.distance;
    }
};

/**
 * @brief Computes a custom measure and optionally colorizes points based on distance.
 *
 * This function calculates the distance from each point in `cloud_b` to its nearest
 * neighbor in `cloud_a` using a k-d tree. The distances are then raised to the power
 * of an exponent, and points that are classified as missing are counted. If the `colorize`
 * flag is set to true, points are colorized based on their distance to the nearest neighbor.
 *
 * @param[in] cloud_a First point cloud.
 * @param[in] cloud_b Second point cloud.
 * @param[in] colorize Flag to control whether points should be colorized based on distance.
 * @return QString A summary of the classified points and the total missing distance.
 */
QString Compute::computeCustomMeasure3(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorize) {
    pcl::search::KdTree<PointT> tree_a;
    tree_a.setInputCloud(cloud_a);

    // Prepare a vector of Data structures to store distances and indices
    vector<Data> values(cloud_b->points.size()); // distance, index A, index B

    // Get the exponent for distance calculation from the UI
    float exponent = mainWindow->getExponent();

    // For each point in cloud_b, find its nearest neighbor in cloud_a
    for (size_t i = 0; i < cloud_b->points.size(); ++i) {
        auto &point = cloud_b->points[i];
        pcl::Indices indices(1);  // To store the index of the nearest point
        vector<float> sqr_distances(1);  // To store the squared distance to the nearest point

        // Perform nearest neighbor search
        tree_a.nearestKSearch(point, 1, indices, sqr_distances);

        // Store the computed distance and indices
        values[i].distance = std::pow(std::sqrt(sqr_distances[0]), exponent);
        values[i].indexA = indices[0];
        values[i].indexB = i;
    }

    // Sort the values based on distance
    sort(values.begin(), values.end());

    // Create a used vector to track which points have been classified
    vector<bool> used(cloud_a->points.size());

    // Find the maximum distance from the computed distances
    auto maxElement = std::max_element(values.begin(), values.end());
    float max_dist = maxElement->distance;

    vector<Data> points;
    float total_dist = 0.00f;

    // Identify the points that are classified as missing
    for (int i = 0; i < values.size(); i++) {
        auto item = values[i];
        if (!used[item.indexA]) {
            used[item.indexA] = true;
        }
        else {
            total_dist += item.distance;
            points.push_back(item);
        }
    }

    for (auto &point : cloud_b->points) {
        point.r = 0;
    }

    // If colorization is enabled, apply colors based on distance
    if (colorize) {
        QColor color;
        for (auto data : points) {
            // Calculate a factor for color intensity based on distance
            float threshold = 0.8f;
            float factor = data.distance / max_dist;

            if (factor <= threshold) {
                factor = factor / threshold;
            }
            else {
                factor = 1.0f;
            }

            // Set the red color intensity based on the factor
            color.setRed(255 * factor);
            cloud_b->points[data.indexB].r = color.red();
        }
    }
    else {
        // If colorization is not enabled, reset color to 0
        for (auto data : points) {
            cloud_b->points[data.indexB].r = 0;
        }
    }

    // Return a summary of the missing points and their total distance
    return QString(
               "Points classified as missing:  %1\n"
               "Total missing distance:  %2\n")
        .arg(points.size())
        .arg(total_dist);
}

