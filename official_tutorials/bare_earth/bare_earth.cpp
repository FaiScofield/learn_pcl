#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/shared_ptr.hpp>

int main (int argc, char** argv)
{
  if (argc < 2) {
    PCL_ERROR ("Usage: bare_earth pcd_file\n");
    return (-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  if (reader.read<pcl::PointXYZ>(argv[1], *cloud) == -1)
      PCL_ERROR ("Couldn't read file! \n");

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  if (cloud->size() > 10000) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud);
  }


  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

//  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZ> ("ground.pcd", *cloud_filtered, false);

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*object);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *object << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->addCoordinateSystem(1.0);

  int v1(0), v2(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("ground", 10, 10, "v1 text", v1);
  viewer->addPointCloud(cloud_filtered, "ground", v1);

  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("object", 10, 10, "v2 text", v2);
  viewer->addPointCloud(object, "object", v2);


  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

//  writer.write<pcl::PointXYZ> ("object.pcd", *cloud_filtered, false);

  return (0);
}

