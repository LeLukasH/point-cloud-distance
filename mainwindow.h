#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QComboBox>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    PointCloudT::Ptr openFile();
    PointCloudT::Ptr transformToRGBA(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz);
    void updateViewer(int id);
    void openFileForViewer1();
    void openFileForViewer2();
    void onSlider1ValueChanged(double value);
    void onSlider2ValueChanged(double value);

    void onCalculate();
    double hausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    double chamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    double earthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);
    double jensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b);

    void showHistogram(const std::vector<float>& distances);
    QBarSeries* createHistogramSeries(const std::vector<float>& distances, int numBins);
    void resetHistogram();

    void colorizeHandler();
    void colorizeCloud(PointCloudT::Ptr &cloud, const std::vector<float>& distances);
    void colorizePoint(PointT &point, float distance, float max_distance, float min_distance);
    QColor calculateColor(float color_factor);

    void saveCurrentView(pcl::visualization::PCLVisualizer::Ptr viewer);
    void deleteView(int index);
    void renameView(int index);
    void applyViewToViewer(pcl::visualization::PCLVisualizer::Ptr viewer, int index);
    void populateViewList();

private:
    double pointSize1 = 1;
    double pointSize2 = 3;

protected:
    PointCloudT::Ptr cloud1;
    PointCloudT::Ptr cloud2;

    pcl::visualization::PCLVisualizer::Ptr viewer1;
    pcl::visualization::PCLVisualizer::Ptr viewer2;

    QChartView* chartView;
    QVector<pcl::visualization::Camera> cameraViews;
    QVector<QString> cameraViewNames;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
