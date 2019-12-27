// Test_kd_tree.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"


//================= convert xyz to pcd =========================//
//#include <pcl/io/pcd_io.h>  
//#include<iostream>  
//using namespace std;
//
//int numofPoints(char* fname)
//{
//	int n = 0;
//	int c = 0;
//	FILE *fp;
//	fp = fopen(fname, "r");
//	do {
//		c = fgetc(fp);
//		if (c == '\n')
//		{
//			++n;
//		}
//	} while (c != EOF);
//	fclose(fp);
//	return n;
//}
//
//int main()
//{
//	int n = 0; //n�������ļ��е����      
//	FILE *fp_1;
//	fp_1 = fopen("data\\horse_xyz.xyz", "r");
//	n = numofPoints("data\\horse_xyz.xyz");//ʹ��numofPoints���������ļ��е����  
//	cout << "there are " << n << " points in the file..." << endl;
//
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	cloud.width = n;
//	cloud.height = 1;
//	cloud.is_dense = false;
//	cloud.points.resize(cloud.width * cloud.height);
//  
//	double x, y, z;
//	int i = 0;
//	while (3 == fscanf(fp_1, "%lf %lf %lf\n", &x, &y, &z))
//	{
//		cloud.points[i].x = x;
//		cloud.points[i].y = y;
//		cloud.points[i].z = z;
//		++i;
//	}
//	fclose(fp_1);
//	//������ָ��ָ������ݴ���pcd�ļ�  
//
//	pcl::io::savePCDFileASCII("data\\horse_xyz.pcd", cloud);
//
//	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
//	
//	system("pause");
//	return 0;
//}


//====================== kd tree search ==========================//
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h> 
#include <iostream>
#include <vector>
#include <ctime>

#include <chrono> 
using namespace std;
using namespace chrono; 

#include <vector>



int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // �������ƣ�ָ�룩

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("data\\horse_xyz.pcd", *cloud) == -1) //* ����PCD��ʽ���ļ�������ļ������ڣ�����-1
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n"); 
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from test_file.pcd with the following fields: "
		<< std::endl;
	/*
	for (size_t i = 0; i < cloud->points.size(); ++i) //��ʾ���еĵ�
													  //for (size_t i = 0; i < cloud->size(); ++i) // Ϊ�˷���۲죬ֻ��ʾǰ5����
		std::cout << "    " << cloud->points[i].x
		<< " " << cloud->points[i].y
		<< " " << cloud->points[i].z << std::endl; 
	*/

	//------------- K nearest neighbor search----------------//
	auto start = system_clock::now();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(cloud);
	int K = 10;
	vector<pcl::PointXYZ> v_points(K*cloud->width);

	pcl::PointXYZ searchPoint;


	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		searchPoint= cloud->points[i];

		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
				v_points[10*i+j] = cloud->points[pointIdxNKNSearch[j]];
			
			//std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			//	<< " " << cloud->points[pointIdxNKNSearch[i]].y
			//	<< " " << cloud->points[pointIdxNKNSearch[i]].z
			//	<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
			
		}
	}
	//��Ҫ��ʱ�����
	auto end = system_clock::now();
	auto duration = duration_cast<microseconds>(end - start);
	printf("(CPU)  cost time: %f ms\n", 1000 * double(duration.count())*microseconds::period::num / microseconds::period::den);

	//-------------Neighbors within radius search------------------//
	
	/*	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;


	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	*/

	return 0;
}
