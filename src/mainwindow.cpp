#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>

#include <opencv2/opencv.hpp>

#include <QFileDialog>
#include <QMessageBox>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QValueAxis>
#include <QtCharts/QBarCategoryAxis>

#include <QListWidget>
#include <QInputDialog>
#include <QStandardItemModel>

#include "compute.h"


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;
using namespace std;
using namespace pcl::visualization;


#include <pcl/visualization/cloud_viewer.h>
#include <vtkGenericOpenGLRenderWindow.h>

/**
 * @class MainWindow
 * @brief The MainWindow class is the main GUI for the application.
 *        It handles user interactions, including loading point clouds, adjusting point sizes,
 *        applying various distance metrics, and managing camera views.
 */
MainWindow* MainWindow::instance = nullptr;
Compute* compute;

/**
 * @brief Constructs a MainWindow object and sets up all necessary UI components and signal connections.
 *
 * Initializes the UI, sets up renderers for the point cloud viewers, and connects various UI elements
 * (e.g., buttons, sliders, combo boxes) to their corresponding slots for user interaction.
 *
 * @param parent The parent QWidget for this window.
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    instance = this;
    compute = new Compute(this);

    cloud1 = nullptr;
    cloud2 = nullptr;

    // Connect buttons and sliders to appropriate slots for user actions
    connect(ui->openButton1, &QPushButton::clicked, this, [=]() { openFileForViewer(1); });
    connect(ui->openButton2, &QPushButton::clicked, this, [=]() { openFileForViewer(2); });
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

    // Connect slider to spin box for point size adjustments
    connect(ui->pointSizeSlider1, &QSlider::valueChanged, [=](int value) { ui->pointSizeBox1->setValue(value / 10.0); });
    connect(ui->pointSizeSlider2, &QSlider::valueChanged, [=](int value) { ui->pointSizeBox2->setValue(value / 10.0); });
    // Connect spin box to slider for point size adjustments
    connect(ui->pointSizeBox1, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) { ui->pointSizeSlider1->setValue(value * 10.0); });
    connect(ui->pointSizeBox2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) { ui->pointSizeSlider2->setValue(value * 10.0); });

    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::onCalculate);

    // Setup the viewers for the point cloud displays
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

    // Setup chart view for the histogram
    chartView = new QChartView();
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setStyleSheet("background: transparent");
    chartView->setMinimumHeight(200);
    chartView->setMaximumHeight(300);
    ui->histogramPlace->addWidget(chartView);
    QChart* chart = new QChart();
    chartView->setChart(chart);
    resetHistogram();

    // Populate the color format options
    QString colorFormats[] = {"RGB", "Yellow-Red", "Grayscale", "CMYK", "Heatmap", "Pastel", "Rainbow", "Default (No color)"};
    for (const auto& format : colorFormats) {
        ui->colorFormatBox->addItem(format);
    }

    cameraViews = QVector<CameraView>();

    // Set default camera views
    Camera camDefault, camTop, camBottom, camLeft, camRight;
    vector<Camera> cameras;
    viewer1->getCameras(cameras);
    camDefault = cameras[0];
    camTop = camDefault;
    camTop.pos[1] = 15.0;
    camTop.view[1] = 0;
    camTop.view[1] = -1;
    camBottom = camDefault;
    camBottom.pos[1] = -15.0;
    camBottom.view[1] = 0;
    camBottom.view[1] = 1;
    camLeft = camDefault;
    camLeft.pos[0] = -15.0;
    camRight = camDefault;
    camRight.pos[0] = 15.0;
    cameraViews.push_back({ "Default", camDefault, 5.0 });
    cameraViews.push_back({ "Top", camTop, 5.0 });
    cameraViews.push_back({ "Bottom", camBottom, 5.0 });
    cameraViews.push_back({ "Left", camLeft, 5.0 });
    cameraViews.push_back({ "Right", camRight, 5.0 });

    updateComboBoxes();

    // Connect camera view combo boxes to corresponding viewer
    connect(ui->cameraViewComboBox1, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { applyViewToViewer(1, index); });
    connect(ui->cameraViewComboBox2, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index) { applyViewToViewer(2, index); });
    connect(ui->saveViewButton1, &QPushButton::clicked, this, [=]() { saveCurrentView(1); });
    connect(ui->deleteViewButton1, &QPushButton::clicked, this, [=]() { deleteCurrentView(ui->cameraViewComboBox1->currentIndex()); });
    connect(ui->saveViewButton2, &QPushButton::clicked, this, [=]() { saveCurrentView(2); });
    connect(ui->deleteViewButton2, &QPushButton::clicked, this, [=]() { deleteCurrentView(ui->cameraViewComboBox2->currentIndex()); });

    // Register mouse callback for the point cloud viewers
    viewer1->registerMouseCallback([this](const pcl::visualization::MouseEvent& event) {
        mouseCallback(event, 1);
    });

    viewer2->registerMouseCallback([this](const pcl::visualization::MouseEvent& event) {
        mouseCallback(event, 2);
    });

    rangeSlider = new RangeSlider(Qt::Horizontal, RangeSlider::Option::DoubleHandles, nullptr);
    rangeSlider->SetRange(0, 100);
    QHBoxLayout *rangeSliderLayout = new QHBoxLayout();
    rangeSliderLayout->setContentsMargins(35, 0, 35, 0); // 20px left and right, no top or bottom margins
    rangeSliderLayout->addWidget(rangeSlider);
    ui->histogramPlace->addLayout(rangeSliderLayout);
    connect(rangeSlider, &RangeSlider::lowerValueChanged, this, [=]() { colorizeHandler(); });
    connect(rangeSlider, &RangeSlider::upperValueChanged, this, [=]() { colorizeHandler(); });

    connect(ui->cleanLogButton, &QPushButton::clicked, this, &MainWindow::clearLog);

    QDoubleValidator *validator = new QDoubleValidator(-9.99, -9.99, 2, this); // Limits: 0 to 1,000,000 with 2 decimal places
    validator->setNotation(QDoubleValidator::StandardNotation);
    ui->exponentInput->setValidator(validator);

    connect(ui->updateButton, &QPushButton::clicked, this, [=]() { colorizeHandler(); });

    connect(ui->exportTargetButton, &QPushButton::clicked, this, [=]() { exportImage(1); });
    connect(ui->exportReferenceButton, &QPushButton::clicked, this, [=]() { exportImage(2); });
}

/**
 * @brief Destructor for the MainWindow object, cleaning up allocated resources.
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * @brief Calculates and displays the selected distance metric between the loaded point clouds.
 *
 * This method checks if both point clouds are loaded, then computes the selected distance metric
 * (Hausdorff, Chamfer, Earth Movers, etc.) and logs the result.
 *
 * @return void
 */
void MainWindow::onCalculate() {
    // Check if the cloud data is loaded and valid
    if (!cloud1 || !cloud2) {
        QMessageBox::warning(this, "Error", "Please load both point clouds first.");
        return;
    }

    QString logMessage;
    // Check which radio button is selected and compute the corresponding distance metric
    if (ui->hausdorffRadioButton->isChecked()) {
        logMessage = compute->computeHausdorffDistance(cloud1, cloud2);
    } else if (ui->chamferRadioButton->isChecked()) {
        logMessage = compute->computeChamferDistance(cloud1, cloud2);
    } else if (ui->earthMoversRadioButton->isChecked()) {
        logMessage = compute->computeEarthMoversDistance(cloud1, cloud2);
    } else if (ui->jensenShannonRadioButton->isChecked()) {
        if (cloud1->size() != cloud2->size()) {
            QMessageBox::warning(this, "Error", "To calculate Jensen-Shannon Divergence, both point clouds must have the same number of points.");
            return;
        }
        logMessage = compute->computeJensenShannonDivergence(cloud1, cloud2);
    }
    else if (ui->custom1RadioButton->isChecked()) {
        logMessage = compute->computeCustomMeasure1(cloud1, cloud2);
    }
    else if (ui->custom2RadioButton->isChecked()) {
        logMessage = compute->computeCustomMeasure2(cloud1, cloud2);
    }
    else if (ui->custom3RadioButton->isChecked()) {
        logMessage = compute->computeCustomMeasure3(cloud1, cloud2, ui->missingDataCheckBox->isChecked());
    }
    else {
        QMessageBox::warning(this, "Error", "Please select a distance metric.");
        return;
    }

    // Append log message to logField
    ui->logField->append("--------------------------------------------------------------\n\n" + logMessage);
}

// ########################################################################
// #################### Colorize ##########################################
// ########################################################################

/**
 * @brief Handles the colorization process for the clouds.
 *
 * This function checks if both clouds are loaded, and based on that, enables or disables relevant UI elements.
 * It computes the distances between the clouds and updates the histogram and the viewer accordingly.
 *
 * @return void
 */
void MainWindow::colorizeHandler() {
    bool cloudsLoaded = (cloud1 != nullptr && cloud2 != nullptr); // Check if both clouds are loaded

    // Enable or disable colorizeButton and colorFormatBox based on whether both clouds are loaded
    ui->colorFormatBox->setEnabled(cloudsLoaded); // Only enable colorFormatBox if colorizeButton is checked and clouds are loaded
    ui->missingDataCheckBox->setEnabled(cloudsLoaded);

    if (cloudsLoaded) {
        vector<float> distances = compute->getDistances(cloud1, cloud2);
        showHistogram(distances);
        colorizeCloud(cloud1, distances);
        compute->computeCustomMeasure3(cloud1, cloud2, ui->missingDataCheckBox->isChecked());
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

/**
 * @brief Calculates the color for a given factor based on the selected color format.
 *
 * This function maps the factor (a normalized value) to a color in the selected color format (e.g., RGB, Grayscale).
 * It applies specific transformations to the factor based on the color format and returns the resulting color.
 *
 * @param factor A normalized value representing the factor for color determination.
 * @return QColor The calculated color corresponding to the factor.
 */
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
    else if (format == "Default (No color)") {
        color = defaultColor;
    }

    return color;
}

/**
 * @brief Colorizes a point based on its distance relative to the maximum and minimum distances.
 *
 * This function normalizes the distance of a point and assigns it a color based on that value using the `colorizePoint` function.
 *
 * @param point The point to colorize.
 * @param distance The distance of the point from the cloud.
 * @param max_distance The maximum distance in the cloud.
 * @param min_distance The minimum distance in the cloud.
 * @return void
 */
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

/**
 * @brief Colorizes a cloud of points based on their respective distances.
 *
 * This function applies color to each point in the cloud by calling `colorizePoint` with each point's distance. It calculates the maximum
 * and minimum distances within the cloud and normalizes the distances for each point.
 *
 * @param cloud The point cloud to colorize.
 * @param distances A vector containing the distances for each point in the cloud.
 * @return void
 */
void MainWindow::colorizeCloud(PointCloudT::Ptr &cloud, const vector<float>& distances){
    double max_dist = *max_element(distances.begin(), distances.end());
    double min_dist = *min_element(distances.begin(), distances.end());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto &point = cloud->points[i];
        colorizePoint(point, distances[i], max_dist, min_dist);
    }
}
// ########################################################################
// ########################## Histogram ###################################
// ########################################################################

/**
 * @brief Creates a histogram series for the given distances.
 *
 * @param distances A vector containing the distances to be plotted in the histogram.
 * @param numBins The number of bins to divide the histogram into.
 * @return A QBarSeries containing the histogram data.
 */
QBarSeries* MainWindow::createHistogramSeries(const std::vector<float>& distances, int numBins) {
    float maxDist = *std::max_element(distances.begin(), distances.end());
    float minDist = *std::min_element(distances.begin(), distances.end());

    // Ensure maxDist is greater than minDist for proper binning
    if (maxDist - minDist == 0) maxDist += 0.005;

    // Calculate the bin width based on the range and number of bins
    float binWidth = (maxDist - minDist) / numBins;

    std::vector<int> bins(numBins, 0);
    // Bin the distances
    for (float dist : distances) {
        int binIndex = std::min(static_cast<int>((dist - minDist) / binWidth), numBins - 1);
        bins[binIndex]++;
    }

    QBarSeries* series = new QBarSeries();

    // Create a bar set and add the bins to the series
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

/**
 * @brief Resets the histogram chart by removing all series and resetting the background.
 */
void MainWindow::resetHistogram() {
    QChart *chart = chartView->chart();

    chart->removeAllSeries();
    chart->setBackgroundBrush(Qt::NoBrush); // Transparent background
    chart->setPlotAreaBackgroundVisible(false); // No background in the plot area
    chart->legend()->hide(); // Hide the legend
    chart->setTitle("Histogram");

    // Set the x-axis range and tick count
    if (!chart->axes(Qt::Horizontal).empty()) chart->removeAxis(chart->axes(Qt::Horizontal).first());
    QValueAxis* axisX = new QValueAxis();
    axisX->setRange(0, 10);
    axisX->setTickCount(11);
    axisX->setGridLineVisible(false);
    chart->addAxis(axisX, Qt::AlignBottom);
}

/**
 * @brief Displays the histogram for the given distances in the chart view.
 *
 * @param distances A vector containing the distances to be plotted in the histogram.
 */
void MainWindow::showHistogram(const std::vector<float>& distances) {
    int numBins = 100; // Set the number of bins for granularity

    QBarSeries* series = createHistogramSeries(distances, numBins);

    float minDist = *std::min_element(distances.begin(), distances.end());
    float maxDist = *std::max_element(distances.begin(), distances.end());

    // Ensure the range is not zero for proper axis scaling
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
}



// ########################################################################
// ################### Point Cloud Handler ################################
// ########################################################################

/**
 * @brief Updates the viewer with the point cloud for the specified viewer ID.
 *
 * @param id The ID of the viewer (1 or 2) to update with the new point cloud.
 */
void MainWindow::updateViewer(int id) {
    if (id == 1) {
        viewer1->removeAllPointClouds();
        if (cloud1 != nullptr) {
            viewer1->addPointCloud(cloud1);
            viewer1->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, pointSize1);
        }
        ui->qvtkWidget1->renderWindow()->Render();
    }
    else if (id == 2) {
        viewer2->removeAllPointClouds();
        if (cloud2 != nullptr) {
            viewer2->addPointCloud(cloud2);
            viewer2->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, pointSize2);
        }
        ui->qvtkWidget2->renderWindow()->Render();
    }
}

/**
 * @brief Opens a file and loads the point cloud for the specified viewer ID.
 *
 * @param id The ID of the viewer (1 or 2) for which to load the point cloud.
 */
void MainWindow::openFileForViewer(int id) {
    PointCloudT::Ptr loaded_cloud = openFile(id);
    if (loaded_cloud != nullptr) {
        id == 1 ? cloud1 = loaded_cloud : cloud2 = loaded_cloud;
        colorizeHandler();
    }
}

/**
 * @brief Opens a file dialog to load a point cloud file and returns the corresponding point cloud.
 *
 * @param id The ID of the viewer (1 or 2) for which to load the point cloud.
 * @return A pointer to the loaded point cloud.
 */
PointCloudT::Ptr MainWindow::openFile(int id) {
    QString title = id == 1 ? "Target" : "Reference"; // Determine the title based on the viewer ID
    // Open file dialog to select PCD, OBJ, or XYZ file
    QString fileName = QFileDialog::getOpenFileName(this, "Open " + title + " Point Cloud File", "", "PCD, OBJ, PLY, and XYZ Files (*.pcd *.obj *.ply *.xyz)");
    if (fileName.isEmpty()) {
        return nullptr; // Return null if no file is selected
    }

    QFileInfo fileInfo(fileName);
    QString fileExtension = fileInfo.suffix().toLower(); // Get the file extension

    // Load the appropriate point cloud based on file extension
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    if (fileExtension == "pcd") {
        if (pcl::io::loadPCDFile<PointXYZ>(fileName.toStdString(), *cloud_xyz) == -1) {
            QMessageBox::critical(this, "Error", "Failed to load the PCD file.");
            return nullptr;
        }
    } else if (fileExtension == "obj") {
        pcl::PolygonMesh mesh;
        if (pcl::io::loadOBJFile(fileName.toStdString(), mesh) == -1) {
            QMessageBox::critical(this, "Error", "Failed to load the OBJ file.");
            return nullptr;
        }
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud_xyz);
    } else if (fileExtension == "xyz") {
        // Load XYZ file manually
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
                continue; // Skip invalid lines in the XYZ file
            }
            cloud_xyz->points.push_back(point);
        }
        cloud_xyz->width = cloud_xyz->points.size();
        cloud_xyz->height = 1;
        cloud_xyz->is_dense = true;
    } else if (fileExtension == "ply") {
        if (pcl::io::loadPLYFile(fileName.toStdString(), *cloud_xyz) == -1) {
            QMessageBox::critical(this, "Error", "Failed to load the PLY file.");
            return nullptr;
        }
    } else {
        QMessageBox::critical(this, "Error", "Unsupported file format.");
        return nullptr;
    }

    QString viewerName = "";
    QLabel* viewerLabel;
    if (id == 1) {
        viewerName = "Target";
        viewerLabel = ui->targetLabel;
    }
    else {
        viewerName = "Reference";
        viewerLabel = ui->referenceLabel;
    }

    int maxLength = 70; // Adjust based on UI space
    if (fileName.length() > maxLength) {
        QString shortened = fileName.left(15) + "..." + fileName.right(50);
        viewerLabel->setText(viewerName + ": " + shortened);
    } else {
        viewerLabel->setText(viewerName + ": " + fileName);
    }

    // Transform PointXYZ to PointXYZRGBA and return the transformed cloud
    return transformToRGBA(cloud_xyz);
}

/**
 * @brief Transforms a PointXYZ cloud to a PointXYZRGBA cloud with default colors.
 *
 * @param cloud_xyz The input point cloud in PointXYZ format.
 * @return A pointer to the transformed PointXYZRGBA cloud.
 */
PointCloudT::Ptr MainWindow::transformToRGBA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz) {
    PointCloudT::Ptr transformed_cloud(new PointCloudT);
    transformed_cloud->points.resize(cloud_xyz->points.size());
    transformed_cloud->width = cloud_xyz->width;
    transformed_cloud->height = cloud_xyz->height;

    for (size_t i = 0; i < cloud_xyz->points.size(); ++i) {
        PointT p;
        p.x = cloud_xyz->points[i].x;
        p.y = cloud_xyz->points[i].y;
        p.z = cloud_xyz->points[i].z;
        p.r = defaultColor.red(); // Default color
        p.g = defaultColor.green();
        p.b = defaultColor.blue();
        p.a = 255; // Fully opaque
        transformed_cloud->points[i] = p;
    }

    return transformed_cloud; // Return the transformed cloud
}

/**
 * @brief Updates the point size and re-renders the viewer for Viewer 1.
 *
 * @param value The new point size for the first viewer.
 */
void MainWindow::onSlider1ValueChanged(double value) {
    pointSize1 = value / 10.0;
    updateViewer(1);
}

/**
 * @brief Updates the point size and re-renders the viewer for Viewer 2.
 *
 * @param value The new point size for the second viewer.
 */
void MainWindow::onSlider2ValueChanged(double value) {
    pointSize2 = value / 10.0;
    updateViewer(2);
}

// ########################################################################
// ####################### Views ##########################################
// ########################################################################

/**
 * @brief Saves the current view from a selected viewer.
 *
 * @param id The ID of the viewer (1 or 2).
 * @details This function saves the current camera parameters and point size as a new view with a unique name entered by the user.
 * The new view is added to the list of camera views, and the combo boxes are updated accordingly.
 */
void MainWindow::saveCurrentView(int id) {
    Camera camera;
    float pointSize;
    int showIndex1;
    int showIndex2;
    QComboBox* activeComboBox;
    QComboBox* notActiveComboBox;

    // Determine which viewer and associated UI elements are used based on the id
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

/**
 * @brief Deletes the currently selected view from the list of camera views.
 *
 * @param index The index of the camera view to be deleted.
 * @details This function deletes the selected camera view from the list, confirms with the user before deleting,
 * and updates the combo boxes to reflect the changes. The selected index in the combo boxes is adjusted as needed.
 */
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

/**
 * @brief Synchronizes both camera view combo boxes with the current list of camera views.
 *
 * @param showIndex1 The index to be set in the first combo box after synchronization.
 * @param showIndex2 The index to be set in the second combo box after synchronization.
 * @details This function clears both combo boxes, repopulates them with the current camera views,
 * and ensures that the selected indices in the combo boxes are correctly updated.
 */
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

/**
 * @brief Applies the selected camera view parameters to the specified viewer.
 *
 * @param id The ID of the viewer (1 or 2).
 * @param index The index of the view to apply.
 * @details This function applies the camera parameters (such as position, orientation, etc.)
 * of the selected view to the corresponding viewer, and triggers a re-render to update the display.
 */
void MainWindow::applyViewToViewer(int id, int index) {
    if (index >= 0 && index < cameraViews.size()) {
        pcl::visualization::PCLVisualizer::Ptr viewer = (id == 1) ? viewer1 : viewer2;
        QComboBox* activeComboBox = (id == 1) ? ui->cameraViewComboBox1 : ui->cameraViewComboBox2;

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

        // Trigger a re-render for the selected viewer
        updateViewer(id);
    }
}

/**
 * @brief Updates the combo box when the camera changes.
 *
 * This method ensures that the "[Custom]" item is added to the combo box if it doesn't exist,
 * and sets it as the current selection for the specified viewer.
 *
 * @param id The ID of the camera view (1 or 2) that has changed.
 * @return void
 */
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

/**
 * @brief Handles mouse events and triggers camera update on mouse release.
 *
 * This method checks if the left mouse button was released and triggers the camera update
 * for the corresponding viewer.
 *
 * @param event The mouse event that triggered the callback.
 * @param viewerID The ID of the viewer (1 or 2) for which the callback is triggered.
 * @return void
 */
void MainWindow::mouseCallback(const MouseEvent& event, int viewerID) {
    // Check if the mouse button was released
    if (event.getType() == MouseEvent::MouseButtonRelease) {
        if (event.getButton() == MouseEvent::LeftButton) {
            // Call your custom function when the left mouse button is released
            onCameraChanged(viewerID);
        }
    }
}

/**
 * @brief Exports a snapshot of the current viewer as a PNG file.
 *
 * This method opens a file save dialog for the user to choose a file path,
 * appends ".png" to the file if no extension is provided, and saves the current viewer's screenshot.
 *
 * @param none
 * @return void
 */
void MainWindow::exportImage(int id) {
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
    if (id == 1) {
        viewer1->saveScreenshot(filePath.toStdString());
    }
    else {
        viewer2->saveScreenshot(filePath.toStdString());
    }


    // Notify the user that the snapshot was exported
    QMessageBox::information(this, "Success", "Snapshot exported successfully!");
}

/**
 * @brief Clears the log field in the UI.
 *
 * This method clears the text in the log field to reset the logging output.
 *
 * @param none
 * @return void
 */
void MainWindow::clearLog() {
    ui->logField->clear(); // Clears the text in the logField
}

/**
 * @brief Retrieves the exponent value from the input field.
 *
 * This method converts the text input in the exponent input field to a float value.
 *
 * @param none
 * @return float The exponent value entered by the user.
 */
float MainWindow::getExponent() {
    return ui->exponentInput->text().toFloat();
}

/**
 * @brief Handles changes in the state of the missing data checkbox.
 *
 * This method triggers a recomputation of the custom measure 3 based on the checkbox state
 * and updates the viewer accordingly.
 *
 * @param arg1 The new state of the checkbox (Qt::Checked or Qt::Unchecked).
 * @return void
 */
void MainWindow::on_missingDataCheckBox_stateChanged(int arg1) {
    compute->computeCustomMeasure3(cloud1, cloud2, arg1 == Qt::Checked);
    updateViewer(2);
}


