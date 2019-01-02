#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <iostream>
#include <boost/thread/thread.hpp>
#include <chrono>

using namespace std;

bool flag = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void)
{
    if (event.getKeySym() == "q" && event.keyDown())
        flag = true;
}


int main(int argc, char** argv)
{
    if (argc < 2) {
        printf(" -- Usage: %s <pointcloud file>\n", argv[0]);
        return -1;
    }

    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());

    // Time cost
    auto t1 = chrono::steady_clock::now();

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
        return -1;
    auto t2 = chrono::steady_clock::now();
    auto dt = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    printf("Time cost of loading PCD file: %f s\n", dt);

    // Note: you would usually perform downsampling now. It has been omitted here
    // for simplicity, but be aware that computation can take a long time.

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(0.3);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);
    auto t3 = chrono::steady_clock::now();
    dt = chrono::duration_cast<chrono::duration<double> >(t3 - t2).count();
    printf("Time cost of normals estimation: %f s\n", dt);

    // Visulization for points and their normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> normal_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    normal_viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "original cloud");
    normal_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.5, "normal");
    normal_viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) NULL);
    while (!flag && !normal_viewer->wasStopped()) {
        normal_viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::seconds(0.1));
    }
    normal_viewer->close();

    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    pfh.setRadiusSearch(0.5);
    pfh.compute(*descriptors);
    auto t4 = chrono::steady_clock::now();
    dt = chrono::duration_cast<chrono::duration<double> >(t4 - t3).count();
    printf("Time cost of PFH estimation: %f s\n", dt);

    // Visulization for PFH
//    pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter();
//    auto a = (*descriptors)[50];
//    vector<double> his(125);
//    for (size_t i=0; i<125; ++i)
//        his[i] = (double) *(a.histogram + i);
////    vector<double> his(a.histogram);
//    plotter->addHistogramData(his, 125);
//    plotter->setTitle("The PFH of Point 50th");
//    plotter->setShowLegend(true);
//    plotter->plot();

//    // Save histogram
//    ofstream ofs;
//    ofs.open("pfh.txt");
//    if (!ofs.is_open())
//        printf("Save pfh.txt file failed...\n");
//    for (size_t i=0; i<descriptors->size(); ++i) {
//        for (size_t j=0; j<125; ++j)
//            ofs << descriptors->points[i].histogram[j] << " ";
//        ofs << "\n";
//    }
//    ofs.close();

    return 0;
}
