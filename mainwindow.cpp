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
    connect(ui->pointSizeSlider1, &QSlider::valueChanged, this, &MainWindow::refreshView1);
    connect(ui->pointSizeSlider2, &QSlider::valueChanged, this, &MainWindow::refreshView2);
    connect(ui->resetButton1, &QPushButton::clicked, [this]() {viewer1->resetCamera();});
    connect(ui->resetButton2, &QPushButton::clicked, [this]() {viewer2->resetCamera();});
    connect(ui->deleteButton1, &QPushButton::clicked, [this]() {viewer1->removeAllPointClouds(); refreshView1();});
    connect(ui->deleteButton2, &QPushButton::clicked, [this]() {viewer2->removeAllPointClouds(); refreshView2();});
    //connect(ui->colorizeButton, &QCheckBox::toggled, [this]() {colorizeCloud(cloud1, ui->)});

    // Connect slider to spin box
    connect(ui->pointSizeSlider1, &QSlider::valueChanged, [=](int value) {ui->pointSizeBox1->setValue(value / 10.0);});
    connect(ui->pointSizeSlider2, &QSlider::valueChanged, [=](int value) {ui->pointSizeBox2->setValue(value / 10.0);});
    // Connect spin box to slider
    connect(ui->pointSizeBox1, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {ui->pointSizeSlider1->setValue(value * 10.0);});
    connect(ui->pointSizeBox2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {ui->pointSizeSlider2->setValue(value * 10.0);});

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

    // ChartView
    chartView = new QChartView();
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setStyleSheet("background: transparent");
    chartView->setMinimumHeight(300);
    ui->histogramPlace->addWidget(chartView);
    QChart* chart = new QChart();
    chart->setBackgroundBrush(Qt::NoBrush); // Transparent background
    chart->setPlotAreaBackgroundVisible(false); // No background in the plot area
    chart->legend()->hide(); // Hide the legend
    chart->setTitle("Histogram"); // No title
    QValueAxis* axisX = new QValueAxis();
    axisX->setRange(0, 10);
    axisX->setTickCount(10);
    axisX->setGridLineVisible(false);
    chart->addAxis(axisX, Qt::AlignBottom);
    chartView->setChart(chart);
    chartView->setMaximumHeight(150);

    // Populate the ColorFormatBox
    ui->colorFormatBox->addItem("White");
    ui->colorFormatBox->addItem("RGB");
}

MainWindow::~MainWindow()
{
    delete ui;
}

vector<float> getDistances(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud (cloud_b);
    vector<float> distances(cloud_a->points.size());
    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1); // To store index of the nearest point
        vector<float> sqr_distances(1); // To store squared distance of the nearest point

        // Perform nearest neighbor search
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);

        // Calculate the distance
        distances[i] = sqrt(sqr_distances[0]);
    }
    return distances;
}

QColor calculateColor(float color_factor, string format = "white") {
    QColor color;
    if (format == "rgb") {
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
    else if (format == "white") {
        color.setRgb(255,255,255);
    }

    return color;
}

void colorizePoint(PointT &point, float distance, float max_distance, float min_distance) {
    // Normalize the distance for coloring (assuming max distance is a threshold)
    float color_factor;
    if (max_distance - min_distance == 0)
        color_factor = 0;
    else
        color_factor = (distance - min_distance) / (max_distance - min_distance);

    QColor color = calculateColor(color_factor, "rgb");
    point.r = color.red();
    point.g = color.green();
    point.b = color.blue();
}

void MainWindow::colorizeCloud(PointCloudT::Ptr &cloud, vector<float> distances){
    double max_dist = *max_element(distances.begin(), distances.end());
    double min_dist = *min_element(distances.begin(), distances.end());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto &point = cloud->points[i];
        colorizePoint(point, distances[i], max_dist, min_dist);
    }
}

double MainWindow::hausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // compare A to B
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double max_dist = *max_element(distances.begin(), distances.end());

    if (colorized) colorizeCloud(cloud_a, distances);
    showHistogram(distances);
    updateViewer(1, cloud_a);

    return max_dist;
}

double MainWindow::chamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // compare A to B
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double sum = 0;
    for (size_t i = 0; i < distances.size(); i++) {
        sum += distances[i] * distances[i];
    }

    if (colorized) colorizeCloud(cloud_a, distances);
    showHistogram(distances);
    updateViewer(1, cloud_a);

    return sum / cloud_a->points.size();
}

double MainWindow::earthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // compare A to B
    return 0;
}

// Function to calculate Kullback-Leibler divergence
double klDivergence(const vector<double> &P, const vector<double> &Q) {
    double kl_divergence = 0.0;

    for (size_t i = 0; i < P.size(); ++i) {
        if (P[i] != 0 && Q[i] != 0) {
            kl_divergence += P[i] * std::log(P[i] / Q[i]);
        }
    }

    return kl_divergence;
}

double jensenShannonDivergenceFromDistributions(const vector<double> &P, const vector<double> &Q) {
    vector<double> M(P.size());
    double js_divergence = 0.0;

    // Calculate M = 1/2 * (P + Q)
    for (size_t i = 0; i < P.size(); ++i) {
        M[i] = 0.5 * (P[i] + Q[i]);
    }

    // Calculate JSD using KL divergence
    js_divergence = 0.5 * klDivergence(P, M) + 0.5 * klDivergence(Q, M);

    return js_divergence;
}

vector<double> normalizeDistribution(const vector<float> &distances) {
    vector<double> distribution(distances.size());
    double sum = 0.0;

    // Calculate sum of distances
    for (double dist : distances) {
        sum += dist;
    }

    // Normalize each distance to form a probability distribution
    for (size_t i = 0; i < distances.size(); ++i) {
        distribution[i] = static_cast<double>(distances[i]) / sum;
    }

    return distribution;
}

// Function to calculate Jensen-Shannon divergence
double MainWindow::jensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized) {
    // Compare cloud A to B
    vector<float> distances_a = getDistances(cloud_a, cloud_b);
    vector<float> distances_b = getDistances(cloud_b, cloud_a);

    // Normalize both distance arrays to create probability distributions
    vector<double> P = normalizeDistribution(distances_a);
    vector<double> Q = normalizeDistribution(distances_b);

    // Calculate the Jensen-Shannon Divergence
    double jsd = jensenShannonDivergenceFromDistributions(P, Q);

    if (colorized) colorizeCloud(cloud_a, distances_a);
    showHistogram(distances_a);
    updateViewer(1, cloud_a);

    return jsd;
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
        float color_factor = (static_cast<float>(i) / numBins);
        QColor color = calculateColor(color_factor, "rgb");

        barSet->setColor(color);
        barSet->setBrush(QBrush(color));
        barSet->setBorderColor(Qt::transparent);
        series->append(barSet);
    }

    return series;
}

void MainWindow::showHistogram(const std::vector<float>& distances) {
    int numBins = 100; // Set the number of bins to 50 for more granularity
    QBarSeries* series = createHistogramSeries(distances, numBins);

    float minDist = *std::min_element(distances.begin(), distances.end());
    float maxDist = *std::max_element(distances.begin(), distances.end());

    // Set bar width to fill space (no gap between bars)
    series->setBarWidth(1.0);

    // Create and customize the chart
    QChart *chart = chartView->chart();
    chart->removeAllSeries();
    chart->addSeries(series);

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
    } else if (ui->jensenShannonRadioButton->isChecked()) {
        distance = jensenShannonDivergence(cloud1, cloud2, colorized);
        QMessageBox::information(this, "Jenses-Shannon Divergence", QString::number(distance));
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
    pointSize1 = value / 10.0;

}

void MainWindow::onSlider2ValueChanged(double value)
{
    pointSize2 = value / 10.0;
}
