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

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarCategoryAxis>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;
using namespace std;


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

    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::onCalculate);

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

    chartView1 = new QChartView();
    chartView1->setRenderHint(QPainter::Antialiasing);
    chartView1->setStyleSheet("background: transparent");
    chartView1->setMinimumHeight(300);
    ui->verticalLayout1->addWidget(chartView1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

double MainWindow::hausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // compare A to B
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud (cloud_b);
    double max_dist_a = -numeric_limits<double>::max ();
    double min_dist_a = numeric_limits<double>::max ();
    vector<float> distances(cloud_a->points.size());
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        vector<float> sqr_distances(1); // To store squared distance of the nearest point

        // Perform nearest neighbor search
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        // Calculate the distance
        double distance = sqrt(sqr_distances[0]);
        distances[i] = distance;

        if (distance > max_dist_a)
            max_dist_a = distance;
        if (distance < min_dist_a)
            min_dist_a = distance;
    }
    if (colorized) {
        for (size_t i = 0; i < cloud_a->points.size(); ++i) {
            auto &point = cloud_a->points[i];
            colorize(point, distances[i], max_dist_a, min_dist_a);
        }
    }
    showHistogram(distances, chartView1);
    updateViewer(1, cloud_a);
    return max_dist_a;
}

double MainWindow::chamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // compare A to B
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud (cloud_b);
    double max_dist_a = -numeric_limits<double>::max ();
    double min_dist_a = numeric_limits<double>::max ();
    double sum = 0;
    vector<double> distances(cloud_a->points.size());

    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        vector<float> sqr_distances(1); // To store squared distance of the nearest point

        // Perform nearest neighbor search
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        // Calculate the distance
        double distance = sqr_distances[0];
        distances[i] = distance;
        sum += distance;

        if (distance > max_dist_a) max_dist_a = distance;
        if (distance < min_dist_a) min_dist_a = distance;
    }
    if (colorized) {
        for (size_t i = 0; i < cloud_a->points.size(); ++i) {
            auto &point = cloud_a->points[i];
            colorize(point, distances[i], max_dist_a, min_dist_a);
        }
    }
    updateViewer(1, cloud_a);
    return sum / cloud_a->points.size();
}

double MainWindow::earthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // compare A to B
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud (cloud_b);
    double max_dist_a = -numeric_limits<double>::max ();
    double min_dist_a = numeric_limits<double>::max ();
    double sum = 0;
    vector<double> distances(cloud_a->points.size());

    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        vector<float> sqr_distances(1); // To store squared distance of the nearest point

        // Perform nearest neighbor search
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        // Calculate the distance
        double distance = sqr_distances[0];
        distances[i] = distance;
        sum += distance;

        if (distance > max_dist_a) max_dist_a = distance;
        if (distance < min_dist_a) min_dist_a = distance;
    }
    if (colorized) {
        for (size_t i = 0; i < cloud_a->points.size(); ++i) {
            auto &point = cloud_a->points[i];
            colorize(point, distances[i], max_dist_a, min_dist_a);
        }
    }
    updateViewer(1, cloud_a);
    return sum / cloud_a->points.size();
}

// Function to create histogram data
QBarSeries* createHistogramSeries(const std::vector<float>& distances, int numBins, bool colorized = true) {
    float maxDist = *std::max_element(distances.begin(), distances.end());
    float minDist = *std::min_element(distances.begin(), distances.end());
    float binWidth = (maxDist - minDist) / numBins;

    std::vector<int> bins(numBins, 0);
    for (float dist : distances) {
        int binIndex = std::min(static_cast<int>((dist - minDist) / binWidth), numBins - 1);
        bins[binIndex]++;
    }

    QBarSeries* series = new QBarSeries();

    // Create a bar set and add the bins
    for (int i = 0; i < numBins; i++) {
        QBarSet* barSet = new QBarSet("");
        *barSet << bins[i];
        QColor color;
        if (colorized) {
            float color_factor = (static_cast<float>(i) / numBins);
            if (color_factor <= 0.25f) {
                // Red to Yellow
                color.setRgb(255, static_cast<int>(255 * (color_factor / 0.25f)), 0);
            } else if (color_factor <= 0.5f) {
                // Yellow to Green
                color.setRgb(static_cast<int>(255 * (1.0f - (color_factor - 0.25f) / 0.25f)), 255, 0);
            } else if (color_factor <= 0.75f) {
                // Green to Cyan
                color.setRgb(0, 255, static_cast<int>(255 * ((color_factor - 0.5f) / 0.25f)));
            } else {
                // Cyan to Blue
                color.setRgb(0, static_cast<int>(255 * (1.0f - (color_factor - 0.75f) / 0.25f)), 255);
            }
        }
        else {
            color.setRgb(255,255,255);
        }
        barSet->setColor(color);
        barSet->setBrush(QBrush(color));
        barSet->setBorderColor(Qt::transparent);
        series->append(barSet);
    }

    return series;
}

void MainWindow::showHistogram(const std::vector<float>& distances, QChartView* chartView) {
    int numBins = 100; // Set the number of bins to 50 for more granularity
    QBarSeries* series = createHistogramSeries(distances, numBins, false);

    float minDist = *std::min_element(distances.begin(), distances.end());
    float maxDist = *std::max_element(distances.begin(), distances.end());

    // Set bar width to fill space (no gap between bars)
    series->setBarWidth(1.0);

    // Create and customize the chart
    QChart* chart = new QChart();
    chart->removeAllSeries();
    chart->addSeries(series);
    chart->setBackgroundBrush(Qt::NoBrush); // Transparent background
    chart->setPlotAreaBackgroundVisible(false); // No background in the plot area
    chart->legend()->hide(); // Hide the legend
    chart->setTitle("Histogram"); // No title

    // Configure the x-axis to have 10 evenly spaced labels
    QValueAxis* axisX = new QValueAxis();
    axisX->setRange(minDist, maxDist);
    axisX->setTickCount(10);
    axisX->setGridLineVisible(false);
    chart->addAxis(axisX, Qt::AlignBottom);

    // Configure y-axis without labels or grid lines
    /*QValueAxis* axisY = new QValueAxis();
    axisY->setGridLineVisible(false);
    axisY->setLabelsVisible(false);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);*/

    chartView->setChart(chart);
}

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
void MainWindow::onCalculate() {
    // Check if the cloud data is loaded and valid
    if (!cloud1 || !cloud2) {
        QMessageBox::warning(this, "Error", "Please load both point clouds first.");
        return;
    }

    bool colorized = true; // Assuming colorization is always on; you can add a checkbox if needed

    double distance;
    if (ui->hausdorffRadioButton->isChecked()) {
        distance = hausdorffDistance(cloud1, cloud2, colorized);
        QMessageBox::information(this, "Hausdorff Distance", QString::number(distance));
    } else if (ui->chamferRadioButton->isChecked()) {
        distance = chamferDistance(cloud1, cloud2, colorized);
        QMessageBox::information(this, "Chamfer Distance", QString::number(distance));
    } else if (ui->earthMoversRadioButton->isChecked()) {
        distance = earthMoversDistance(cloud1, cloud2, colorized);
        QMessageBox::information(this, "Earth Mover's Distance", QString::number(distance));
    } else {
        QMessageBox::warning(this, "Error", "Please select a distance metric.");
    }

    updateViewer(1, cloud1);
    updateViewer(2, cloud2);
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<PointXYZ>(fileName.toStdString(), *cloud_xyz) == -1) {
                QMessageBox::critical(this, "Error", "Failed to load the PCD file.");
                return nullptr; // Return null if loading fails
        }
        // Transform PointXYZ to PointT (PointXYZRGBA)
        return transformToRGBA(cloud_xyz);

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
        ifstream infile(fileName.toStdString());
        if (!infile) {
            QMessageBox::critical(this, "Error", "Failed to open the XYZ file.");
            return nullptr;
        }

        string line;
        while (getline(infile, line)) {
            istringstream iss(line);
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
