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

#include <opencv2/opencv.hpp>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarCategoryAxis>

#include <QListWidget>
#include <QInputDialog>
#include <QStandardItemModel>



using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;
using namespace std;
using namespace pcl::visualization;


#include <pcl/visualization/cloud_viewer.h>
#include <vtkGenericOpenGLRenderWindow.h>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    cloud1 = nullptr;
    cloud2 = nullptr;

    connect(ui->openButton1, &QPushButton::clicked, this, [=]() {openFileForViewer(1);});
    connect(ui->openButton2, &QPushButton::clicked, this, [=]() {openFileForViewer(2);});
    connect(ui->pointSizeSlider1, &QSlider::valueChanged, this, &MainWindow::onSlider1ValueChanged);
    connect(ui->pointSizeSlider2, &QSlider::valueChanged, this, &MainWindow::onSlider2ValueChanged);

    connect(ui->deleteButton1, &QPushButton::clicked, [this]() {
        cloud1 = nullptr;
        colorizeHandler();
    });
    connect(ui->deleteButton2, &QPushButton::clicked, [this]() {
        cloud2 = nullptr;
        colorizeHandler();
    });
    connect(ui->colorFormatBox, &QComboBox::currentTextChanged, [this]() {
        colorizeHandler();
    });

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
    viewer1.reset(new PCLVisualizer(renderer1, renderWindow1, "viewer1", false));
    ui->qvtkWidget1->setRenderWindow(viewer1->getRenderWindow());
    viewer1->setupInteractor(ui->qvtkWidget1->interactor(), ui->qvtkWidget1->renderWindow());

    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer2.reset(new PCLVisualizer(renderer2, renderWindow2, "viewer2", false));
    ui->qvtkWidget2->setRenderWindow(viewer2->getRenderWindow());
    viewer2->setupInteractor(ui->qvtkWidget2->interactor(), ui->qvtkWidget2->renderWindow());

    viewer1->setShowFPS(0);
    viewer2->setShowFPS(0);
    viewer1->setBackgroundColor(backgroundColor.red(), backgroundColor.green(), backgroundColor.blue());
    viewer2->setBackgroundColor(backgroundColor.red(), backgroundColor.green(), backgroundColor.blue());

    // ChartView
    chartView = new QChartView();
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setStyleSheet("background: transparent");
    chartView->setMinimumHeight(200);
    chartView->setMaximumHeight(300);
    ui->histogramPlace->addWidget(chartView);
    QChart* chart = new QChart();
    chartView->setChart(chart);
    resetHistogram();


    // Populate the ColorFormatBox
    QString colorFormats[] = {"RGB", "Yellow-Red", "Grayscale", "CMYK", "Heatmap", "Pastel", "Rainbow", "Default"};
    for (const auto& format : colorFormats) {
        ui->colorFormatBox->addItem(format);
    }



    cameraViews = QVector<CameraView>();

    // Set default camera parameters (adjust these as needed for your default view)
    Camera camDefault, camTop, camBottom, camLeft, camRight;
    camDefault.pos[2] = 15.0;
    camTop.pos[1] = 15.0;
    camTop.view[1] = 0;
    camTop.view[1] = -1;
    camBottom.pos[1] = -15.0;
    camBottom.view[1] = 0;
    camBottom.view[1] = 1;
    camLeft.pos[0] = -15.0;
    camRight.pos[0] = 15.0;
    cameraViews.push_back({ "Default", camDefault, 2.0 });
    cameraViews.push_back({ "Top", camTop, 2.0 });
    cameraViews.push_back({ "Bottom", camBottom, 2.0 });
    cameraViews.push_back({ "Left", camLeft, 2.0 });
    cameraViews.push_back({ "Right", camRight, 2.0 });

    updateComboBoxes();

    connect(ui->cameraViewComboBox1, QOverload<int>::of(&QComboBox::currentIndexChanged),this, [=](int index) {applyViewToViewer(1, index);});
    connect(ui->cameraViewComboBox2, QOverload<int>::of(&QComboBox::currentIndexChanged),this, [=](int index) {applyViewToViewer(2, index);});
    connect(ui->saveViewButton1, &QPushButton::clicked, this, [=]() {saveCurrentView(1);});
    connect(ui->deleteViewButton1, &QPushButton::clicked, this, [=]() {deleteCurrentView(ui->cameraViewComboBox1->currentIndex());});
    connect(ui->saveViewButton2, &QPushButton::clicked, this, [=]() {saveCurrentView(2);});
    connect(ui->deleteViewButton2, &QPushButton::clicked, this, [=]() {deleteCurrentView(ui->cameraViewComboBox2->currentIndex());});

    viewer1->registerMouseCallback([this](const pcl::visualization::MouseEvent& event) {
            mouseCallback(event, 1);
        });

    viewer2->registerMouseCallback([this](const pcl::visualization::MouseEvent& event) {
        mouseCallback(event, 2);
    });


    rangeSlider = new RangeSlider(Qt::Horizontal, RangeSlider::Option::DoubleHandles, nullptr);
    rangeSlider->SetRange(0,100);
    QHBoxLayout *rangeSliderLayout = new QHBoxLayout();
    rangeSliderLayout->setContentsMargins(35, 0, 35, 0); // 20px left and right, no top or bottom margins
    rangeSliderLayout->addWidget(rangeSlider);
    ui->histogramPlace->addLayout(rangeSliderLayout);
    connect(rangeSlider, &RangeSlider::lowerValueChanged, this, [=]() {colorizeHandler();});
    connect(rangeSlider, &RangeSlider::upperValueChanged, this, [=]() {colorizeHandler();});

    connect(ui->cleanLogButton, &QPushButton::clicked, this, &MainWindow::clearLog);

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

// ########################################################################
// ################### Distances ##########################################
// ########################################################################

void MainWindow::onCalculate() {
    // Check if the cloud data is loaded and valid
    if (!cloud1 || !cloud2) {
        QMessageBox::warning(this, "Error", "Please load both point clouds first.");
        return;
    }

    QString logMessage;
    if (ui->hausdorffRadioButton->isChecked()) {
        logMessage = computeHausdorffDistance(cloud1, cloud2);
    } else if (ui->chamferRadioButton->isChecked()) {
        logMessage = computeChamferDistance(cloud1, cloud2);
    } else if (ui->earthMoversRadioButton->isChecked()) {
        logMessage = computeEarthMoversDistance(cloud1, cloud2);
    } else if (ui->jensenShannonRadioButton->isChecked()) {
        if (cloud1->size() != cloud2->size()) {
            QMessageBox::warning(this, "Error", "To calculate Jensen-Shannon Divergence, both point clouds must have the same number of points.");
            return;
        }
        logMessage = computeJensenShannonDivergence(cloud1, cloud2);
    } else {
        QMessageBox::warning(this, "Error", "Please select a distance metric.");
        return;
    }

    // Append log message to logField
    ui->logField->append(logMessage);
}

QString MainWindow::computeHausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    // compare A to B
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double max_dist = *max_element(distances.begin(), distances.end());

    return QString(
               "Hausdorff Distance computed\n"
               "\n"
               "value : %1\n")
        .arg(max_dist);
}


QString MainWindow::computeChamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    // compare A to B
    vector<float> distances = getDistances(cloud_a, cloud_b);

    double sum = 0;
    for (size_t i = 0; i < distances.size(); i++) {
        sum += distances[i] * distances[i];
    }

    return QString(
               "Chamfer Distance computed\n"
               "\n"
               "value : %1\n")
        .arg(sum / cloud_a->points.size());
}

QString MainWindow::computeEarthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    // Convert point clouds to OpenCV Mat with weights
    cv::Mat signature1(cloud_a->size(), 4, CV_32F); // [weight, x, y, z]
    cv::Mat signature2(cloud_b->size(), 4, CV_32F); // [weight, x, y, z]

    // Fill in the data for signature1
    for (size_t i = 0; i < cloud_a->size(); ++i) {
        signature1.at<float>(i, 0) = 1.0f; // Uniform weight
        signature1.at<float>(i, 1) = cloud_a->points[i].x;
        signature1.at<float>(i, 2) = cloud_a->points[i].y;
        signature1.at<float>(i, 3) = cloud_a->points[i].z;
    }

    // Fill in the data for signature2
    for (size_t i = 0; i < cloud_b->size(); ++i) {
        signature2.at<float>(i, 0) = 1.0f; // Uniform weight
        signature2.at<float>(i, 1) = cloud_b->points[i].x;
        signature2.at<float>(i, 2) = cloud_b->points[i].y;
        signature2.at<float>(i, 3) = cloud_b->points[i].z;
    }

    // Normalize weights (optional, but ensures consistent results)
    signature1.col(0) /= cv::sum(signature1.col(0))[0];
    signature2.col(0) /= cv::sum(signature2.col(0))[0];

    // Calculate EMD
    float emd = cv::EMD(signature1, signature2, cv::DIST_L2);

    return QString(
               "Earth Mover's Distance computed\n"
               "\n"
               "value : %1\n")
        .arg(static_cast<double>(emd));
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

    if (sum != 0) {
        // Normalize each distance to form a probability distribution
        for (size_t i = 0; i < distances.size(); ++i) {
            distribution[i] = static_cast<double>(distances[i]) / sum;
        }
    }

    return distribution;
}

// Function to calculate Jensen-Shannon divergence
QString MainWindow::computeJensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {

    if (cloud_a->size() != cloud_b->size()) return QString("Point clouds do not have the same number of points.");

    // Compare cloud A to B
    vector<float> distances_a = getDistances(cloud_a, cloud_b);
    vector<float> distances_b = getDistances(cloud_b, cloud_a);

    // Normalize both distance arrays to create probability distributions
    vector<double> P = normalizeDistribution(distances_a);
    vector<double> Q = normalizeDistribution(distances_b);

    // Calculate the Jensen-Shannon Divergence
    double jsd = jensenShannonDivergenceFromDistributions(P, Q);

    return QString(
               "Jensen-Shannon Divergence computed\n"
               "\n"
               "value : %1\n")
        .arg(jsd);
}


// ########################################################################
// #################### Colorize ##########################################
// ########################################################################

void MainWindow::colorizeHandler() {
    bool cloudsLoaded = (cloud1 != nullptr && cloud2 != nullptr); // Check if both clouds are loaded

    // Enable or disable colorizeButton and colorFormatBox based on whether both clouds are loaded
    ui->colorFormatBox->setEnabled(cloudsLoaded); // Only enable colorFormatBox if colorizeButton is checked and clouds are loaded
    //rangeSlider->setEnabled(cloudsLoaded);

    if (cloudsLoaded) {
        vector<float> distances = getDistances(cloud1, cloud2);
        showHistogram(distances);
        colorizeCloud(cloud1, distances);
    } else {
        // If one or both clouds are missing, ensure colorizeButton and colorFormatBox are disabled
        //ui->colorizeButton->setChecked(false);
        resetHistogram(); // Reset the chart
        if (cloud1 != nullptr) {
            colorizeCloud(cloud1, vector<float>(cloud1->size()));
        }
    }
    updateViewer(1);
    updateViewer(2);
}


QColor MainWindow::calculateColor(float factor) {
    QColor color;
    float color_factor = factor;

    // Fetch the min and max values from the range slider
    float minThreshold = (rangeSlider->GetLowerValue() - 1) / 100.0f;
    float maxThreshold = rangeSlider->GetUpperValue() / 100.0f;

    if (factor < minThreshold) {
        color_factor = 0.0f;  // Everything below the minimum is 0
    } else if (factor > maxThreshold) {
        color_factor = 1.0f;  // Everything above the maximum is 1
    } else {
        // Normalize within the range [minThreshold, maxThreshold]
        color_factor = (factor - minThreshold) / (maxThreshold - minThreshold);
    }

    QString format = ui->colorFormatBox->currentText();
    if (!ui->colorFormatBox->isEnabled()) format = "Default";

    if (format == "RGB") {
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
    else if (format == "Yellow-Red") {
        // Yellow to Red gradient
        color.setRgb(255, static_cast<int>(255 * (1.0f - color_factor)), 0);
    }
    else if (format == "Grayscale") {
        // Grayscale: Black (0) to White (1)
        int gray = static_cast<int>(255 * color_factor);
        color.setRgb(gray, gray, gray);
    }
    else if (format == "CMYK") {
        // CMYK color space transformation (approximation)
        float c = 1 - color_factor;
        float m = color_factor * 0.75f;
        float y = 0.5f * (1 - color_factor);
        float k = 0.2f;

        int r = static_cast<int>((1 - c) * (1 - k) * 255);
        int g = static_cast<int>((1 - m) * (1 - k) * 255);
        int b = static_cast<int>((1 - y) * (1 - k) * 255);
        color.setRgb(r, g, b);
    }
    else if (format == "Heatmap") {
        // Heatmap: Blue (0) to Red (1)
        color.setRgb(static_cast<int>(255 * color_factor), 0, static_cast<int>(255 * (1 - color_factor)));
    }
    else if (format == "Pastel") {
        // Pastel tones for a softer gradient
        color.setRgb(255, static_cast<int>(200 + 55 * color_factor), static_cast<int>(200 + 55 * (1 - color_factor)));
    }
    else if (format == "Rainbow") {
        // Rainbow: full spectrum using HSV
        int hue = static_cast<int>(color_factor * 360);
        color.setHsv(hue, 255, 255);
    }
    else if (format == "White") {
        color.setRgb(255,255,255);
    }
    else if (format == "Default") {
        color = defaultColor;
    }

    return color;
}

void MainWindow::colorizePoint(PointT &point, float distance, float max_distance, float min_distance) {
    // Normalize the distance for coloring (assuming max distance is a threshold)
    float color_factor;
    if (max_distance - min_distance == 0)
        color_factor = 0;
    else
        color_factor = (distance - min_distance) / (max_distance - min_distance);

    QColor color = calculateColor(color_factor);
    point.r = color.red();
    point.g = color.green();
    point.b = color.blue();
}

void MainWindow::colorizeCloud(PointCloudT::Ptr &cloud, const vector<float>& distances){
    double max_dist = *max_element(distances.begin(), distances.end());
    double min_dist = *min_element(distances.begin(), distances.end());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto &point = cloud->points[i];
        colorizePoint(point, distances[i], max_dist, min_dist);
    }
}




// Function to create histogram data
QBarSeries* MainWindow::createHistogramSeries(const std::vector<float>& distances, int numBins) {
    float maxDist = *std::max_element(distances.begin(), distances.end());
    float minDist = *std::min_element(distances.begin(), distances.end());

    if (maxDist - minDist == 0) maxDist += 0.005;

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
        QColor color = calculateColor(color_factor);

        barSet->setColor(color);
        barSet->setBrush(QBrush(color));
        barSet->setBorderColor(Qt::transparent);
        series->append(barSet);
    }

    return series;
}

void MainWindow::resetHistogram() {
    QChart *chart = chartView->chart();

    chart->removeAllSeries();
    chart->setBackgroundBrush(Qt::NoBrush); // Transparent background
    chart->setPlotAreaBackgroundVisible(false); // No background in the plot area
    chart->legend()->hide(); // Hide the legend
    chart->setTitle("Histogram");

    if (!chart->axes(Qt::Horizontal).empty()) chart->removeAxis(chart->axes(Qt::Horizontal).first());
    QValueAxis* axisX = new QValueAxis();
    axisX->setRange(0, 10);
    axisX->setTickCount(11);
    axisX->setGridLineVisible(false);
    chart->addAxis(axisX, Qt::AlignBottom);
}

void MainWindow::showHistogram(const std::vector<float>& distances) {
    int numBins = 100; // Set the number of bins to 50 for more granularity

    QBarSeries* series = createHistogramSeries(distances, numBins);

    float minDist = *std::min_element(distances.begin(), distances.end());
    float maxDist = *std::max_element(distances.begin(), distances.end());

    if (maxDist - minDist == 0) maxDist += 0.005;

    // Set bar width to fill space (no gap between bars)
    series->setBarWidth(1.0);

    // Create and customize the chart
    QChart *chart = chartView->chart();
    chart->removeAllSeries();
    chart->addSeries(series);

    // Configure the x-axis to have 10 evenly spaced labels
    QValueAxis* axisX = new QValueAxis();
    axisX->setRange(minDist, maxDist);
    axisX->setTickCount(11);
    axisX->setGridLineVisible(false);
    chart->removeAxis(chart->axes(Qt::Horizontal).first());
    chart->addAxis(axisX, Qt::AlignBottom);

    // Configure y-axis without labels or grid lines
    /*QValueAxis* axisY = new QValueAxis();
    axisY->setGridLineVisible(false);
    axisY->setLabelsVisible(false);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisY);*/
}


// ########################################################################
// ################### Point Cloud Handler ################################
// ########################################################################

void MainWindow::updateViewer(int id) {
    if (id == 1) {
        viewer1->removeAllPointClouds();
        if (cloud1 != nullptr) {
            viewer1->addPointCloud(cloud1);
            viewer1->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, pointSize1);
        }
        ui->qvtkWidget1->renderWindow()->Render();

    }
    else if (id == 2) {
        viewer2->removeAllPointClouds();
        if (cloud2 != nullptr) {
            viewer2->addPointCloud(cloud2);
            viewer2->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, pointSize2);
        }
        ui->qvtkWidget2->renderWindow()->Render();

    }
}

void MainWindow::openFileForViewer(int id)
{
    PointCloudT::Ptr loaded_cloud = openFile();
    if (loaded_cloud != nullptr) {
        id == 1 ? cloud1 = loaded_cloud : cloud2 = loaded_cloud;
        colorizeHandler();
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
        p.r = defaultColor.red(); // Default color value
        p.g = defaultColor.green();
        p.b = defaultColor.blue();
        p.a = 255;
        transformed_cloud->points[i] = p;
    }

    return transformed_cloud; // Return the transformed cloud
}

void MainWindow::onSlider1ValueChanged(double value)
{
    pointSize1 = value / 10.0;
    updateViewer(1);

}

void MainWindow::onSlider2ValueChanged(double value)
{
    pointSize2 = value / 10.0;
    updateViewer(2);
}



// ########################################################################
// ####################### Views ##########################################
// ########################################################################

void MainWindow::saveCurrentView(int id) {
    Camera camera;
    float pointSize;
    int showIndex1;
    int showIndex2;
    QComboBox* activeComboBox;
    QComboBox* notActiveComboBox;

    if (id == 1) {
        viewer1->getCameraParameters(camera);
        pointSize = pointSize1;
        showIndex1 = cameraViews.size();
        showIndex2 = ui->cameraViewComboBox2->currentIndex();
        activeComboBox = ui->cameraViewComboBox1;
        notActiveComboBox = ui->cameraViewComboBox2;
    }
    else if (id == 2) {
        viewer2->getCameraParameters(camera);
        pointSize = pointSize2;
        showIndex1 = ui->cameraViewComboBox1->currentIndex();
        showIndex2 = cameraViews.size();
        activeComboBox = ui->cameraViewComboBox2;
        notActiveComboBox = ui->cameraViewComboBox1;
    }
    else {
        return;
    }

    // Loop until a unique name is entered or the user cancels
    bool ok;
    QString newName;
    do {
        newName = QInputDialog::getText(this, tr("Save View"),
                                        tr("Enter a unique name for the view:"), QLineEdit::Normal,
                                        tr("View %1").arg(cameraViews.size()), &ok);

        if (!ok || newName.isEmpty()) {
            return; // User canceled or entered an empty name, so exit without saving
        }

        // Check if the entered name already exists in the cameraViews list
        bool nameExists = false;
        for (const auto& view : cameraViews) {
            if (view.name == newName) {
                QMessageBox::warning(this, tr("Duplicate Name"),
                                     tr("A view with this name already exists. Please choose a different name."));
                nameExists = true;
                break;
            }
        }

        if (!nameExists) {
            break; // Name is unique, exit loop to proceed with saving
        }
    } while (true);

    // Create and add the new view if the name is unique
    CameraView view;
    view.name = newName;
    view.camera = camera;
    view.pointSize = pointSize;

    cameraViews.push_back(view); // Add the new view to the list

    // Remove "[Custom]" if it exists
    int customIndex = activeComboBox->findText("[Custom]");
    if (customIndex != -1) {
        activeComboBox->removeItem(customIndex);
    }
    if (notActiveComboBox->findText("[Custom]") != -1) {
        if (id == 1) showIndex2++;
        else showIndex1++;
    }

    updateComboBoxes(showIndex1, showIndex2); // Refresh the ComboBoxes
}

void MainWindow::deleteCurrentView(int index) {
    if (index > 0 && index < cameraViews.size()) {
        QMessageBox::StandardButton confirm = QMessageBox::question(this, "Delete View",
                                                                    tr("Are you sure you want to delete %1?").arg(cameraViews[index].name));

        if (confirm == QMessageBox::Yes) {
            cameraViews.removeAt(index);

            int showIndex1 = ui->cameraViewComboBox1->currentIndex();
            int showIndex2 = ui->cameraViewComboBox2->currentIndex();

            // If the deleted index is the selected index in either ComboBox, set that ComboBox to "[Custom]"
            if (showIndex1 == index) {
                if (ui->cameraViewComboBox1->findText("[Custom]") == -1) {
                    ui->cameraViewComboBox1->addItem("[Custom]");
                }
                showIndex1 = ui->cameraViewComboBox1->findText("[Custom]");
            }
            if (showIndex2 == index) {
                if (ui->cameraViewComboBox2->findText("[Custom]") == -1) {
                    ui->cameraViewComboBox2->addItem("[Custom]");
                }
                showIndex2 = ui->cameraViewComboBox2->findText("[Custom]");
            }

            // Adjust indexes if needed (if an item before them was deleted)
            if (showIndex1 > index) showIndex1--;
            if (showIndex2 > index) showIndex2--;

            updateComboBoxes(showIndex1, showIndex2);
        }
    }
}

// Sync combo boxes with current views
void MainWindow::updateComboBoxes(int showIndex1, int showIndex2) {
    int customPosition1 = ui->cameraViewComboBox1->findText("[Custom]");
    int customPosition2 = ui->cameraViewComboBox1->findText("[Custom]");

    // Clear and reset both combo boxes
    ui->cameraViewComboBox1->clear();
    ui->cameraViewComboBox2->clear();

    if (showIndex1 > cameraViews.size()) showIndex1 = cameraViews.size();
    if (showIndex2 > cameraViews.size()) showIndex2 = cameraViews.size();

    for (const auto& view : cameraViews) {
        ui->cameraViewComboBox1->addItem(view.name);
        ui->cameraViewComboBox2->addItem(view.name);
    }

    if (customPosition1 != -1) {
        ui->cameraViewComboBox1->addItem("[Custom]");
    }
    if (customPosition2 != -1) ui->cameraViewComboBox2->addItem("[Custom]");

    ui->cameraViewComboBox1->setCurrentIndex(showIndex1);
    ui->cameraViewComboBox2->setCurrentIndex(showIndex2);
}

void MainWindow::applyViewToViewer(int id, int index) {
    if (index >= 0 && index < cameraViews.size()) {
        pcl::visualization::PCLVisualizer::Ptr viewer = (id == 1) ? viewer1 : viewer2;
        //QSlider* pointSizeSlider = (id == 1) ? ui->pointSizeSlider1 : ui->pointSizeSlider2;
        QComboBox* activeComboBox = (id == 1) ? ui->cameraViewComboBox1 : ui->cameraViewComboBox2;;

        // Remove "[Custom]" if it exists
        int customIndex = activeComboBox->findText("[Custom]");
        if (customIndex != -1) {
            activeComboBox->removeItem(customIndex);
        }

        // Save current viewport dimensions to maintain resolution
        int originalWidth = viewer->getRenderWindow()->GetSize()[0];
        int originalHeight = viewer->getRenderWindow()->GetSize()[1];

        // Apply the camera parameters
        viewer->setCameraParameters(cameraViews[index].camera);

        // Restore the original viewport dimensions if they changed
        viewer->getRenderWindow()->SetSize(originalWidth, originalHeight);

        // Update point size slider
        //pointSizeSlider->setValue(cameraViews[index].pointSize * 10.0);

        // Trigger a re-render for the selected viewer
        updateViewer(id);
    }
}

// Function to update the combo box when the camera changes
void MainWindow::onCameraChanged(int id) {
    QComboBox* comboBox = (id == 1) ? ui->cameraViewComboBox1 : ui->cameraViewComboBox2;;

    // Add "[Custom]" if not already present
    int customIndex = comboBox->findText("[Custom]");
    if (customIndex == -1) {
        comboBox->addItem("[Custom]");
        customIndex = comboBox->count() - 1;  // Last index
    }

    // Set "[Custom]" as the selected item
    comboBox->setCurrentIndex(customIndex);
}

// Mouse callback function
void MainWindow::mouseCallback(const MouseEvent& event, int viewerID) {
    // Check if the mouse button was released
    if (event.getType() == MouseEvent::MouseButtonRelease) {
        if (event.getButton() == MouseEvent::LeftButton) {
            // Call your custom function when the left mouse button is released
            onCameraChanged(viewerID);
        }
    }
}


void MainWindow::on_exportButton_clicked() {
    // Open file save dialog
    QString filePath = QFileDialog::getSaveFileName(
        this,
        "Export Snapshot",
        "snapshot000.png",
        "PNG File (*.png);;All Files (*)"
        );

    if (filePath.isEmpty()) {
        return; // User canceled the dialog
    }

    // Append .png if no extension is provided
    QFileInfo fileInfo(filePath);
    if (fileInfo.suffix().isEmpty()) {
        filePath += ".png";
    }

    // Export PNG
    viewer1->saveScreenshot(filePath.toStdString());


    QMessageBox::information(this, "Success", "Snapshot exported successfully!");
}

void MainWindow::clearLog() {
    ui->logField->clear(); // Clears the text in the logField
}
