/*
 * check_input.cpp
 *
 *  Created on: Jul 9, 2014
 *      Author: thomio
 */
//=============================================================================================================
#include "3d_features.hpp"
//=============================================================================================================
// InputCloud constructor
//=============================================================================================================
InputCloud::InputCloud(){
	// source_cloud = new pcl::PointCloud<pcl::PointXYZ>();
	source_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	file_is_pcd = false;
}
//=============================================================================================================
// InputCloud destructor
//=============================================================================================================
InputCloud::~InputCloud(){
	//delete source_cloud;
}
//=============================================================================================================
// output help messages
//=============================================================================================================
int InputCloud::showHelp(char *program_name){
	std::cout << std::endl;
	std::cout << " Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << " -h: Show this help." << std::endl;
	return 0;
}
//=============================================================================================================
// Check the input dataset sanity
//=============================================================================================================
int InputCloud::checkInput(int argc,char** argv)
{
	// Show help - if asked for it or in case of failure to open the input file
	if(pcl::console::find_switch(argc,argv,"-h") || pcl::console::find_switch(argc,argv,"--help"))
	{
		showHelp(argv[0]);
		std::exit(EXIT_SUCCESS);
	}

	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	filenames = pcl::console::parse_file_extension_argument(argc,argv,".ply");
	if(filenames.size() != 1)
	{
		filenames = pcl::console::parse_file_extension_argument(argc,argv,".pcd");

		if(filenames.size() != 1)
		{
			showHelp(argv[0]);
			std::exit(EXIT_FAILURE);
		} else
		{
			file_is_pcd = true;
		}
	}
	return 0;	// return check_input()
}
//=============================================================================================================
// load dataset into memory
//=============================================================================================================
int InputCloud::loadDataset(char** argv)
{
	// Load file | Works with PCD and PLY files
	if(file_is_pcd) {
		if(pcl::io::loadPCDFile(argv[filenames[0]],*source_cloud) < 0)
		{
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp(argv[0]);
			std::exit(EXIT_FAILURE);
		}
	} else {
		if(pcl::io::loadPLYFile(argv[filenames[0]],*source_cloud) < 0)
		{
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp (argv[0]);
			std::exit(EXIT_FAILURE);
		}
	}
	return 0;
}
//=============================================================================================================
//
//=============================================================================================================
int InputCloud::calc3dFeatures(void)
{
	// pass the input dataset to a normal estimation object
	ne.setInputCloud(source_cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object
	// Its content will be filled inside the object, based on the given input dataset ( as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);

	// Compute the features
	ne.compute(*cloud_normals);

	// cloud_normals->points.size() should have the same size as the input cloud->size()

	return 0;
}
//=============================================================================================================
//
//=============================================================================================================
int InputCloud::visualizeCloud(void){
	// Visualization
	pcl::visualization::PCLVisualizer viewer(" Point Cloud Datsets Visualizer");

	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud,255,255,255);

	viewer.setBackgroundColor(0.05,0.05,0.05,0); // Set background to a dark grey

	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(source_cloud,cloud_normals);
	viewer.addPointCloud(source_cloud,source_cloud_color_handler,"original_cloud");
	

	while(!viewer.wasStopped()){   // Display the visualizer until the 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
//=============================================================================================================
