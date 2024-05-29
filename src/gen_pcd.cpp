#include <array>
#include <iostream>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

static const char *pcd_file = "four_circle_boundary_80x80.pcd";
static int width = 80;
static int height = 80;
static std::array<pcl::PointXY, 4> centers = {{
    {15.f, 15.f},
    {-15.f, 15.f},
    {15.f, -15.f},
    {-15.f, -15.f},
}};
static float radius = 12;

int main() {
  std::vector<pcl::PointXY> points{
      // Four vertices
      {width * .5f, height * .5f},
      {width * -.5f, height * .5f},
      {width * .5f, height * -.5f},
      {width * -.5f, height * -.5f},
  };

  // Horizontal edges
  for (int i = 1; i < width; i++) {
    points.push_back({i - width * .5f, height * .5f});
    points.push_back({i - width * .5f, height * -.5f});
  }

  // Vertical edges
  for (int i = 1; i < height; i++) {
    points.push_back({width * .5f, i - height * .5f});
    points.push_back({width * -.5f, i - height * .5f});
  }

  // Circles
  for (auto c : centers) {
    // Sample every 6 degrees
    for (int d = 0; d < 360; d += 6) {
      points.push_back({c.x + radius * cosf(d * (M_PI / 180)),
                        c.y + radius * sinf(d * (M_PI / 180))});
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = points.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.resize(cloud.width * cloud.height);
  for (int i = 0; i < points.size(); i++) {
    cloud.at(i).x = points.at(i).x / 100;
    cloud.at(i).y = points.at(i).y / 100;
    cloud.at(i).z = 0;
  }
  pcl::PCDWriter w;
  w.writeASCII<pcl::PointXYZ>(pcd_file, cloud, 6);
  std::cout << "Saved " << cloud.size() << " data points to test_pcd.pcd."
            << std::endl;
  return (0);
}
