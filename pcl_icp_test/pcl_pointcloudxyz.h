#include <jni.h>

#ifndef _Included_pcl_pointcloudxyz
#define _Included_pcl_pointcloudxyz
#ifdef __cplusplus
extern "C" {
#endif
	
	JNIEXPORT void JNICALL Java_pcl_pointcloudxyz_alloc
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointcloudxyz_dispose
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointcloudxyz_nGet
	(JNIEnv *, jobject, jint, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointcloudxyz_add
	(JNIEnv *, jobject, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointcloudxyz_remove
	(JNIEnv *, jobject, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointcloudxyz_clear
	(JNIEnv *, jobject);

	JNIEXPORT jint JNICALL Java_pcl_pointcloudxyz_size
	(JNIEnv *, jobject);

	JNIEXPORT jboolean JNICALL Java_pcl_pointcloudxyz_isOrganized
	(JNIEnv *, jobject);

#ifdef __cplusplus
}
#endif
#endif
