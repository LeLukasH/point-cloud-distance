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
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        std::vector<float> sqr_distances(1); // To store squared distance of the nearest point

        // Perform nearest neighbor search
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        // Get the squared distance
        float distance_squared = sqr_distances[0];

        // Update maximum distance if necessary
        if (distance_squared > max_dist_a) {
            max_dist_a = distance_squared;
        }

        point.r = 150;
        std::cout << point;
/*
        // Color the point based on distance
        float distance = std::sqrt(distance_squared); // Euclidean distance
        uint8_t r = 0, g = 0, b = 0; // Initialize color components

        // Normalize the distance to a range (you can customize this range)
        float normalized_distance = std::min(distance / 5.0f, 1.0f); // Assume max distance for colorization is 5.0f

        // Color based on distance
        if (normalized_distance < 0.5f) {
            // Green to yellow
            r = static_cast<uint8_t>(normalized_distance * 2 * 255); // From 0 to 255
            g = 255; // Keep green at max
        } else {
            // Yellow to red
            r = 255; // Keep red at max
            g = static_cast<uint8_t>((1.0f - normalized_distance) * 255); // From 255 to 0
        }

        // Assign color to the point
        cloud_a->points[i].r = r;
        cloud_a->points[i].g = g;
        cloud_a->points[i].b = b; // Set blue to 0
        cloud_a->points[i].a = 255; // Set alpha to fully opaque*/

        //std::cout << cloud_a->points[i].r;
    }

    return max_dist_a;
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

void MainWindow::openFileForViewer1()
{
    cloud1 = openFile();

    if (cloud1) {
        viewer1->removeAllPointClouds();
        viewer1->addPointCloud(cloud1, "cloud1");
    }
}

void MainWindow::openFileForViewer2()
{
    cloud2 = openFile();

    if (cloud2) {
        viewer2->removeAllPointClouds();
        viewer2->addPointCloud(cloud2, "cloud2");
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
