# Point Cloud Quality Visualization Tool

This project is a C++ Windows desktop application for visualizing and analyzing the quality of 3D point clouds. It provides an easy-to-use graphical interface for computing distances and other quality measures between point clouds and visualizing differences through various colorization techniques.

---

## 🚀 Getting Started

You can use the precompiled version without building it yourself.

```bash
git clone https://github.com/LeLukasH/point-cloud-distance.git
```

### ✅ Quick Start (Precompiled)

1. Download the `exe/` folder.
2. Run the `PointCloudQuality.exe` file.

No installation required.

---

## 🛠 Features

- Compare two 3D point clouds visually and quantitatively
- Supported distance metrics:
  - Hausdorff Distance
  - Chamfer Distance
  - Earth Mover’s Distance
  - Jensen-Shannon Divergence
  - Identity Measure (custom)
  - Outliers Measure (custom)
  - Missing Data Measure (custom)
- View distribution of distances using histogram
- Multiple colorization schemes: RGB, Yellow-Red, Grayscale, Heatmap, CMYK, Pastel, Rainbow
- Adjustable point size for each cloud
- Load file formats: `.pcd`, `.ply`, `.obj`, `.xyz`
- Dual viewer interface (target and reference cloud)
- Save and switch between camera views
- **Export visualizations to PNG images** (saves a snapshot of the current viewer's display)

For a more detailed explanation of the features and their implementation, please refer to the end of Chapter 3 in my [bachelor thesis](path-to-bachelor-thesis.pdf).

---

## 📦 Dependencies

This project uses the following libraries:

- [Qt Framework](https://www.qt.io/)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [VTK (via PCL)](https://vtk.org/)
- [OpenCV](https://opencv.org/)

---

## 📷 Screenshot
![app_screen3](https://github.com/user-attachments/assets/d344cd11-833a-4127-820d-26294cd22ad4)

---

## 🧠 How It Works

The core idea is to compute the shortest distance from each point in one cloud to the nearest point in the other using a KD-tree (from PCL). This information is used to:
- Visualize color-coded distances
- Compute similarity measures

### Example: Core distance calculation

```cpp
std::vector<float> getDistances(PointCloudT::Ptr &cloud_a, PointCloudT::Ptr &cloud_b) {
    pcl::search::KdTree<PointT> tree_b;
    tree_b.setInputCloud(cloud_b);

    std::vector<float> distances(cloud_a->points.size());

    for (size_t i = 0; i < cloud_a->points.size(); ++i) {
        auto &point = cloud_a->points[i];
        pcl::Indices indices(1);
        std::vector<float> sqr_distances(1);
        tree_b.nearestKSearch(point, 1, indices, sqr_distances);
        distances[i] = std::sqrt(sqr_distances[0]);
    }
    return distances;
}
```

---

## 🔍 Comparison Methods

- **Hausdorff Distance**: Maximum of all point-to-point distances.
- **Chamfer Distance**: Average of all distances.
- **Earth Mover’s Distance (EMD)**: Measures the cost of transforming one distribution into another. Implemented using OpenCV’s `EMD` function.
- **Jensen-Shannon Divergence**: Measures similarity between histograms.

---

## 🤝 Contributing

Pull requests are welcome. For major changes, please open an issue first.

---

## 📬 Contact

For any questions, feel free to contact me.

---
