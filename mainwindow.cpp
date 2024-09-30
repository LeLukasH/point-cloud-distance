#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>

#include <pcl/io/pcd_io.h>


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
    connect(ui->pointSizeSlider1, &QSlider::sliderReleased, this, &MainWindow::refreshView);
    connect(ui->pointSizeSlider2, &QSlider::sliderReleased, this, &MainWindow::refreshView);

    //viewer2.setPosition(0, 500);

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

void MainWindow::refreshView() {
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize1, "cloud1");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize2, "cloud2");
    ui->qvtkWidget1->renderWindow()->Render();
    ui->qvtkWidget2->renderWindow()->Render();
}

void MainWindow::openFileForViewer1()
{
    cloud1 = openFile();

    if (cloud1) {
        // Process the cloud further if needed
        QMessageBox::information(this, "Success", "PCD file loaded successfully with " +
                                                      QString::number(cloud1->points.size()) + " points.");
        viewer1->addPointCloud(cloud1, "cloud1");
    }


}

void MainWindow::openFileForViewer2()
{
    cloud2 = openFile();

    if (cloud2) {
        // Process the cloud further if needed
        QMessageBox::information(this, "Success", "PCD file loaded successfully with " +
                                                      QString::number(cloud2->points.size()) + " points.");
        viewer2->addPointCloud(cloud2, "cloud2");

    }
}

PointCloudT::Ptr MainWindow::openFile()
{
    // Open file dialog to select PCD file
    QString fileName = QFileDialog::getOpenFileName(this, "Open PCD File", "", "PCD Files (*.pcd)");
    if (fileName.isEmpty()) {
        return nullptr; // Return null if no file is selected
    }

    // Load the PCD file
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(fileName.toStdString(), *cloud) == -1) {
        QMessageBox::critical(this, "Error", "Failed to load the PCD file.");
        return nullptr; // Return null if loading fails
    }

    // Return the loaded point cloud
    return cloud;
}

void MainWindow::onSlider1ValueChanged(double value)
{
    pointSize1 = value;

}

void MainWindow::onSlider2ValueChanged(double value)
{
    pointSize2 = value;
}
