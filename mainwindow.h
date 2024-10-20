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
    void colorizeCloud(PointCloudT::Ptr &cloud, std::vector<float> distances);
    void openFileForViewer1();
    void openFileForViewer2();
    void onSlider1ValueChanged(double value);
    void onSlider2ValueChanged(double value);

    void onCalculate();
    double hausdorffDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    double chamferDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    double earthMoversDistance(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    double jensenShannonDivergence(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b, bool colorized = false);
    void showHistogram(const std::vector<float>& distances);

private:
    double pointSize1 = 1;
    double pointSize2 = 3;

protected:
    PointCloudT::Ptr cloud1;
    PointCloudT::Ptr cloud2;

    pcl::visualization::PCLVisualizer::Ptr viewer1;
    pcl::visualization::PCLVisualizer::Ptr viewer2;

    void refreshView1();
    void refreshView2();

    QChartView* chartView;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
