#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;


#include <pcl/visualization/cloud_viewer.h>
#include <vtkGenericOpenGLRenderWindow.h>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->openButton1, &QPushButton::clicked, this, &MainWindow::openFileForViewer1);
    connect(ui->openButton2, &QPushButton::clicked, this, &MainWindow::openFileForViewer2);
    connect(ui->pointSizeSlider1, &QSlider::valueChanged, this, &MainWindow::onSlider1ValueChanged);
    connect(ui->pointSizeSlider2, &QSlider::valueChanged, this, &MainWindow::onSlider2ValueChanged);
    connect(ui->pointSizeSlider1, &QSlider::sliderMoved, this, &MainWindow::refreshView1);
    connect(ui->pointSizeSlider2, &QSlider::sliderMoved, this, &MainWindow::refreshView2);

    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::onCalculateHausdorffDistance);

    auto renderer1 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow1 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow1->AddRenderer(renderer1);
    viewer1.reset(new pcl::visualization::PCLVisualizer(renderer1, renderWindow1, "viewer1", false));
    ui->qvtkWidget1->setRenderWindow(viewer1->getRenderWindow());
    viewer1->setupInteractor(ui->qvtkWidget1->interactor(), ui->qvtkWidget1->renderWindow());

    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer2.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer2", false));
    ui->qvtkWidget2->setRenderWindow(viewer2->getRenderWindow());
    viewer2->setupInteractor(ui->qvtkWidget2->interactor(), ui->qvtkWidget2->renderWindow());

}

MainWindow::~MainWindow()
{
    delete ui;
}

double MainWindow::hausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    // compare A to B
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud (cloud_b);
    float max_dist_a = -std::numeric_limits<float>::max ();
    float min_dist_a = std::numeric_limits<float>::max ();
    std::vector<float> distances(cloud_a->points.size());
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        std::vector<float> sqr_distances(1); // To store squared distance of the nearest point

        // Perform nearest neighbor search
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        // Calculate the distance
        float distance = std::sqrt(sqr_distances[0]);
        distances[i] = distance;

        if (distance > max_dist_a)
            max_dist_a = distance;
        if (distance < min_dist_a)
            min_dist_a = distance;
    }
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        colorize(point, distances[i], max_dist_a, min_dist_a);
    }
    updateViewer(1, cloud_a);
    return max_dist_a;
}

/*
double MainWindow::pointToPlaneDistance(PointCloudT::Ptr &cloud_a, pcl::PolygonMesh::Ptr &mesh_b) {
    // Extract the cloud from the PolygonMesh
    PointCloudT::Ptr cloud_b(new PointCloudT);
    pcl::fromPCLPointCloud2(mesh_b->cloud, *cloud_b);

    float max_dist_a = -std::numeric_limits<float>::max();
    float min_dist_a = std::numeric_limits<float>::max();
    std::vector<float> distances(cloud_a->points.size());

    // Iterate over each point in cloud_a
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        float min_distance = std::numeric_limits<float>::max();

        // Iterate over each triangle in the mesh
        for (size_t j = 0; j < mesh_b->polygons.size(); ++j) {
            const pcl::Vertices &vertices = mesh_b->polygons[j];

            // Extract triangle vertices (v0, v1, v2) from cloud_b
            Eigen::Vector3f v0(cloud_b->points[vertices.vertices[0]].x, cloud_b->points[vertices.vertices[0]].y, cloud_b->points[vertices.vertices[0]].z);
            Eigen::Vector3f v1(cloud_b->points[vertices.vertices[1]].x, cloud_b->points[vertices.vertices[1]].y, cloud_b->points[vertices.vertices[1]].z);
            Eigen::Vector3f v2(cloud_b->points[vertices.vertices[2]].x, cloud_b->points[vertices.vertices[2]].y, cloud_b->points[vertices.vertices[2]].z);

            // Compute the distance from the point to the current triangle
            float distance = pointToTriangleDistance(point, v0, v1, v2);

            // Track the minimum distance for this point
            if (distance < min_distance) {
                min_distance = distance;
            }
        }

        // Store the minimum distance for this point
        distances[i] = min_distance;

        // Update the max and min distances
        if (min_distance > max_dist_a) max_dist_a = min_distance;
        if (min_distance < min_dist_a) min_dist_a = min_distance;
    }

    // Colorize the points in cloud_a based on their distances to the nearest plane
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        colorize(point, distances[i], max_dist_a, min_dist_a);
    }

    // Update the viewer to show the colorized point cloud
    updateViewer(1, cloud_a);

    return max_dist_a;
}
*/
void MainWindow::colorize(PointT &point, float distance, float max_distance, float min_distance) {
    // Normalize the distance for coloring (assuming max distance is a threshold)
    float color_factor;
    if (max_distance - min_distance == 0)
        color_factor = 0;
    else
        color_factor = (distance - min_distance) / (max_distance - min_distance);

    // Set color based on the normalized distance using the red -> yellow -> green -> cyan -> blue gradient
    if (color_factor <= 0.25f) {
        // Transition from Red to Yellow (0.0 to 0.25)
        point.r = 255;  // Red stays at max
        point.g = static_cast<uint8_t>(255 * (color_factor / 0.25f)); // Green increases
        point.b = 0;    // Blue remains 0
    } else if (color_factor <= 0.5f) {
        // Transition from Yellow to Green (0.25 to 0.5)
        point.r = static_cast<uint8_t>(255 * (1.0f - (color_factor - 0.25f) / 0.25f)); // Red decreases
        point.g = 255;  // Green stays at max
        point.b = 0;    // Blue remains 0
    } else if (color_factor <= 0.75f) {
        // Transition from Green to Cyan (0.5 to 0.75)
        point.r = 0;    // Red is 0
        point.g = 255;  // Green stays at max
        point.b = static_cast<uint8_t>(255 * ((color_factor - 0.5f) / 0.25f)); // Blue increases
    } else {
        // Transition from Cyan to Blue (0.75 to 1.0)
        point.r = 0;    // Red stays at 0
        point.g = static_cast<uint8_t>(255 * (1.0f - (color_factor - 0.75f) / 0.25f)); // Green decreases
        point.b = 255;  // Blue stays at max
    }
}
void MainWindow::onCalculateHausdorffDistance() {
    if (!cloud1 || !cloud2) {
        QMessageBox::warning(this, "Warning", "Please load both point clouds first.");
        return;
    }

    double distance = hausdorffDistance(cloud1, cloud2);
    QMessageBox::information(this, "Hausdorff Distance", "The Hausdorff distance is: " + QString::number(distance));
}

void MainWindow::refreshView1() {
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize1, "cloud1");
    ui->qvtkWidget1->renderWindow()->Render();
}

void MainWindow::refreshView2() {
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize2, "cloud2");
    ui->qvtkWidget2->renderWindow()->Render();
}

void MainWindow::updateViewer(int id, PointCloudT::Ptr cloud) {
    if (id == 1) {
        viewer1->removeAllPointClouds();
        viewer1->addPointCloud(cloud, "cloud1");
        refreshView1();
    }
    else if (id == 2) {
        viewer2->removeAllPointClouds();
        viewer2->addPointCloud(cloud, "cloud2");
        refreshView2();
    }
}

void MainWindow::openFileForViewer1()
{
    cloud1 = openFile();

    if (cloud1) {
        updateViewer(1, cloud1);
    }
}

void MainWindow::openFileForViewer2()
{
    cloud2 = openFile();

    if (cloud2) {
        updateViewer(2, cloud2);
    }
}

PointCloudT::Ptr MainWindow::openFile()
{
    // Open file dialog to select PCD, OBJ, or XYZ file
    QString fileName = QFileDialog::getOpenFileName(this, "Open PCD, OBJ, or XYZ File", "", "PCD, OBJ, and XYZ Files (*.pcd *.obj *.xyz)");
    if (fileName.isEmpty()) {
        return nullptr; // Return null if no file is selected
    }

    QFileInfo fileInfo(fileName);
    QString fileExtension = fileInfo.suffix().toLower();

    if (fileExtension == "pcd") {
        // Try to load as PointT (PointXYZRGBA)
        PointCloudT::Ptr cloud_rgba(new PointCloudT);

        if (pcl::io::loadPCDFile<PointT>(fileName.toStdString(), *cloud_rgba) == -1) {
            // If failed, try loading as PointXYZ
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud_xyz) == -1) {
                QMessageBox::critical(this, "Error", "Failed to load the PCD file.");
                return nullptr; // Return null if loading fails
            }

            // Transform PointXYZ to PointT (PointXYZRGBA)
            return transformToRGBA(cloud_xyz);
        }

        // Return the loaded PointT (PointXYZRGBA) cloud
        return cloud_rgba;

    } else if (fileExtension == "obj") {
        // Load the OBJ file as a polygon mesh
        pcl::PolygonMesh mesh;
        if (pcl::io::loadOBJFile(fileName.toStdString(), mesh) == -1) {
            QMessageBox::critical(this, "Error", "Failed to load the OBJ file.");
            return nullptr; // Return null if loading fails
        }

        // Extract vertices from the mesh and convert to PointT (PointXYZRGBA)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud_xyz);

        // Transform PointXYZ to PointT (PointXYZRGBA)
        return transformToRGBA(cloud_xyz);

    } else if (fileExtension == "xyz") {
        // Load the XYZ file
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        std::ifstream infile(fileName.toStdString());
        if (!infile) {
            QMessageBox::critical(this, "Error", "Failed to open the XYZ file.");
            return nullptr;
        }

        std::string line;
        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            pcl::PointXYZ point;
            if (!(iss >> point.x >> point.y >> point.z)) {
                continue; // Skip invalid lines
            }
            cloud_xyz->points.push_back(point);
        }

        cloud_xyz->width = cloud_xyz->points.size();
        cloud_xyz->height = 1; // XYZ files are usually organized as unorganized point clouds
        cloud_xyz->is_dense = true;

        // Transform PointXYZ to PointT (PointXYZRGBA)
        return transformToRGBA(cloud_xyz);

    } else {
        QMessageBox::critical(this, "Error", "Unsupported file format.");
        return nullptr;
    }
}

PointCloudT::Ptr MainWindow::transformToRGBA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz)
{
    // Transform PointXYZ to PointT (PointXYZRGBA)
    PointCloudT::Ptr transformed_cloud(new PointCloudT);
    transformed_cloud->points.resize(cloud_xyz->points.size());
    transformed_cloud->width = cloud_xyz->width;
    transformed_cloud->height = cloud_xyz->height;

    for (size_t i = 0; i < cloud_xyz->points.size(); ++i) {
        PointT p;
        p.x = cloud_xyz->points[i].x;
        p.y = cloud_xyz->points[i].y;
        p.z = cloud_xyz->points[i].z;
        p.r = 255; // Default color value
        p.g = 255;
        p.b = 255;
        p.a = 255;
        transformed_cloud->points[i] = p;
    }

    return transformed_cloud; // Return the transformed cloud
}

void MainWindow::onSlider1ValueChanged(double value)
{
    pointSize1 = value;

}

void MainWindow::onSlider2ValueChanged(double value)
{
    pointSize2 = value;
}
