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

struct CameraView {
    QString name;
    pcl::visualization::Camera camera;
    float pointSize; // or float if point size can be fractional
};

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

    void saveCurrentView(int id);
    void deleteCurrentView(int index);
    void updateComboBoxes(int showIndex1 = 0, int showIndex2 = 0);
    void applyViewToViewer(int id, int index);
    void onCameraChanged(int id);
    void mouseCallback(const pcl::visualization::MouseEvent& event, int viewerID);

    void printCamera(const pcl::visualization::Camera& camera);


private:
    double pointSize1 = 1;
    double pointSize2 = 3;

protected:
    PointCloudT::Ptr cloud1;
    PointCloudT::Ptr cloud2;

    pcl::visualization::PCLVisualizer::Ptr viewer1;
    pcl::visualization::PCLVisualizer::Ptr viewer2;

    QChartView* chartView;
    QVector<CameraView> cameraViews;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
