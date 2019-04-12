#include "pcl_pointxyz.h"
#include "sptr_wrapper.h"

#include <pcl/point_types.h>

void Java_pcl_pointxyz_alloc(JNIEnv *env, jobject obj)
{
	pcl::PointXYZ *pt_ptr = new pcl::PointXYZ();

	set_handle(env, obj, pt_ptr);
}

void Java_pcl_pointxyz_dispose
(JNIEnv *env, jobject obj)
{
	pcl::PointXYZ *pt_ptr = get_handle<pcl::PointXYZ>(env, obj);

	delete pt_ptr;

	set_handle<pcl::PointXYZ>(env, obj, nullptr);
}

jfloat Java_pcl_pointxyz_getX(JNIEnv *env, jobject obj)
{
	return get_handle<pcl::PointXYZ>(env, obj)->x;
}

void Java_pcl_pointxyz_setX(JNIEnv *env, jobject obj, jfloat x)
{
	get_handle<pcl::PointXYZ>(env, obj)->x = x;
}

jfloat Java_pcl_pointxyz_getY(JNIEnv *env, jobject obj)
{
	return get_handle<pcl::PointXYZ>(env, obj)->y;
}

void Java_pcl_pointxyz_setY(JNIEnv *env, jobject obj, jfloat y)
{
	get_handle<pcl::PointXYZ>(env, obj)->y = y;
}

jfloat Java_pcl_pointxyz_getZ(JNIEnv *env, jobject obj)
{
	return get_handle<pcl::PointXYZ>(env, obj)->z;
}

void Java_pcl_pointxyz_setZ(JNIEnv *env, jobject obj, jfloat z)
{
	get_handle<pcl::PointXYZ>(env, obj)->z = z;
}


