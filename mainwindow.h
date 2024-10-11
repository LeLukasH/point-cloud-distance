#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QtCharts/QChartView>

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
    void updateViewer(int id, PointCloudT::Ptr cloud);
    void colorize(PointT &point, float distance, float max_distance, float min_distance);
    void openFileForViewer1();
    void openFileForViewer2();
    void onSlider1ValueChanged(double value);
    void onSlider2ValueChanged(double value);

    void onCalculate();
    double hausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    double chamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    double earthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    void showDistanceHistogram(const std::vector<float>& distances, QChartView* chartView);

private:
    double pointSize1 = 3;
    double pointSize2 = 3;

protected:
    PointCloudT::Ptr cloud1;
    PointCloudT::Ptr cloud2;

    pcl::visualization::PCLVisualizer::Ptr viewer1;
    pcl::visualization::PCLVisualizer::Ptr viewer2;

    void refreshView1();
    void refreshView2();

    QChartView* chartView1;
    QChartView* chartView2;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
