﻿/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_movlad_pcl_Point2d */

#ifndef _Included_pcl_icp
#define _Included_pcl_icp
#ifdef __cplusplus
extern "C" {
#endif

	JNIEXPORT void JNICALL Java_pcl_icp_alloc
	(JNIEnv *, jobject);

	JNIEXPORT jfloat JNICALL Java_pcl_icp_compute
	(JNIEnv *, jobject, jobject, jobject, jint, jfloat, jfloatArray,jfloatArray);

	JNIEXPORT void JNICALL Java_pcl_icp_transform
	(JNIEnv *, jobject, jobject, jobject);

	JNIEXPORT jfloat JNICALL Java_pcl_icp_test
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_icp_dispose
	(JNIEnv *, jobject);

#ifdef __cplusplus
}
#endif
#endif
