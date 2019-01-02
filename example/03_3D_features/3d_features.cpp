/*
 *  3d_features.cpp
 *	BRAZILIAN INTITUTE OF ROBOTICS
 *	Created on: Jul 7, 2014
 *  Author: thomio watanabe
 */
//=============================================================================================================
#include "3d_features.hpp"
//=============================================================================================================
int main(int argc,char **argv)
{
	InputCloud pc_dataset;

	// Check input data sanity
	pc_dataset.checkInput(argc,argv);
	// Load data into memory
	pc_dataset.loadDataset(argv);
	
	pc_dataset.calc3dFeatures();
	
	// Visualize the source cloud data set
	pc_dataset.visualizeCloud();

	

	return 0;
}

//=============================================================================================================
