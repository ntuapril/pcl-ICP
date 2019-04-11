#include "pcl_icp.h"
#include "handle.h"
#include "sptr_wrapper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <jni.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

void Java_pcl_icp_alloc
(JNIEnv *env, jobject obj)
{
	sptr_wrapper<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> *icp_ptr_w =
		new sptr_wrapper<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>;
	
	icp_ptr_w->instantiate(env, obj);
	set_handle(env, obj, icp_ptr_w);
}

void Java_pcl_icp_dispose
(JNIEnv *env, jobject obj)
{
	sptr_wrapper<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>::dispose(env, obj);
}

jfloat Java_pcl_icp_compute
(JNIEnv *env, jobject obj, jobject src, jobject dst, jint iter, jfloat threshold, jfloatArray matrix, jfloatArray output)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_ptr =
		sptr_wrapper<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>::get_sptr(env, obj);

	pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr
		= sptr_wrapper< pcl::PointCloud<pcl::PointXYZ> >::get_sptr(env, src);

	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_ptr
		= sptr_wrapper< pcl::PointCloud<pcl::PointXYZ> >::get_sptr(env, dst);

	dst_ptr->is_dense = false;
	src_ptr->is_dense = false;

	icp_ptr->setInputSource(src_ptr);
	icp_ptr->setInputTarget(dst_ptr);

	icp_ptr->setEuclideanFitnessEpsilon(threshold);
	icp_ptr->setMaximumIterations(iter);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp_ptr->align(Final);
	icp_ptr->getFinalTransformation();

	Eigen::Matrix<float, 4, 4> mat = icp_ptr->getFinalTransformation();

	jfloat fill[16];
	memcpy(fill, mat.data(), 16 * sizeof(float));
	env->SetFloatArrayRegion(matrix, 0, 16, fill);
	
	int finalsize = src_ptr->size();

	float* finalarray = new float[finalsize * 3];
	float* p = finalarray;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = Final.begin(); it != Final.end(); it++) {
		*p++ = it->x;
		*p++ = it->y;
		*p++ = it->z;
	}	
	env->SetFloatArrayRegion(output, 0, finalsize * 3, finalarray);

	std::cout << "has converged:" << icp_ptr->hasConverged() << " score: " <<
		icp_ptr->getFitnessScore() << std::endl;
	std::cout << icp_ptr->getFinalTransformation() << std::endl;

	return icp_ptr->getFitnessScore();
}

jfloat JNICALL Java_pcl_icp_test
(JNIEnv *env, jobject obj)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_ptr =
		sptr_wrapper<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>::get_sptr(env, obj);
	return 0.0f;
}

void JNICALL Java_pcl_icp_transform
(JNIEnv *env, jobject src, jobject matrix, jobject target)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_ptr
		= sptr_wrapper< pcl::PointCloud<pcl::PointXYZ> >::get_sptr(env, src);

	boost::shared_ptr<Eigen::Matrix4f> matrix_ptr
		= sptr_wrapper<Eigen::Matrix4f>::get_sptr(env, matrix);

	sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>> *target_ptr_w =
		new sptr_wrapper<pcl::PointCloud<pcl::PointXYZ>>();
	target_ptr_w->instantiate(env, target);

	pcl::transformPointCloud(*src_ptr, *target_ptr_w->get_sptr(), *matrix_ptr);
}