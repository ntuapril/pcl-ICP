#include "pcl_pointcloudxyz.h"
#include "sptr_wrapper.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void Java_pcl_pointcloudxyz_alloc(JNIEnv *env, jobject obj)
{
	sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>> *cloud_ptr_w =
		new sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>();

	cloud_ptr_w->instantiate(env, obj);
}

void Java_pcl_pointcloudxyz_dispose
(JNIEnv *env, jobject obj)
{
	sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>::dispose(env, obj);
}

void Java_pcl_pointcloudxyz_nGet(JNIEnv *env, jobject obj, jint i, jobject point)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>::get_sptr(env, obj);

	set_handle<pcl::PointXYZ>(env, point, &(cloud_ptr->at(i)));
}

void Java_pcl_pointcloudxyz_add(JNIEnv *env, jobject obj, jobject point)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>::get_sptr(env, obj);

	cloud_ptr->push_back(*get_handle<pcl::PointXYZ>(env, point));
}

void Java_pcl_pointcloudxyz_remove(JNIEnv *env, jobject obj, jobject point)
{
	throw "Not yet implemented.";
}

void Java_pcl_pointcloudxyz_clear(JNIEnv *env, jobject obj)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>::get_sptr(env, obj);

	cloud_ptr->clear();
}

jint Java_pcl_pointcloudxyz_size(JNIEnv *env, jobject obj)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>::get_sptr(env, obj);

	return cloud_ptr->size();
}

jboolean Java_pcl_pointcloudxyz_isOrganized(JNIEnv *env, jobject obj)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>::get_sptr(env, obj);
	bool isOrganized = cloud_ptr->isOrganized();

	return isOrganized;
}