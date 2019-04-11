#include <pcl/point_cloud.h>
#include "com_movlad_pcl_ICP.h"
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "sptr_wrapper.h"



void Java_com_movlad_pcl_SampleConsensus_nGetInliners
(JNIEnv *env, jobject obj, jobject source, jobject target)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>::Ptr icp =
		sptr_wrapper<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>::get_sptr(env, obj);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_ptr =
		sptr_wrapper<pcl::PointCloud<pcl::PointXYZRGB>>::get_sptr(env, source);

	sptr_wrapper<pcl::PointCloud<pcl::PointXYZRGB>> *target_ptr_w =
		new sptr_wrapper<pcl::PointCloud<pcl::PointXYZRGB>>();

	std::vector<int> inliners;

	target_ptr_w->instantiate(env, target);
	icp->setInputSource();

	pcl::copyPointCloud(*source_ptr, inliners, *target_ptr_w->get_sptr());
}


void Java_com_movlad_pcl_IterativeClosestPoint_alloc
(JNIEnv *env, jobject obj)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> *icp_ptr = new pcl::IterativeClosestPoint();

	set_handle(env, obj, icp_ptr);
}

void Java_com_movlad_pcl_Point2d_dispose
(JNIEnv *env, jobject obj)
{
	pcl::PointXY *pt_ptr = get_handle<pcl::PointXY>(env, obj);

	delete pt_ptr;

	set_handle<pcl::PointXY>(env, obj, nullptr);
}