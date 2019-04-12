#include <jni.h>

#ifndef _Included_pcl_pointxyz
#define _Included_pcl_pointxyz
#ifdef __cplusplus
extern "C" {
#endif

	JNIEXPORT void JNICALL Java_pcl_pointxyz_alloc
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointxyz_dispose
	(JNIEnv *, jobject);

	JNIEXPORT jfloat JNICALL Java_pcl_pointxyz_getX
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointxyz_setX
	(JNIEnv *, jobject, jfloat);

	JNIEXPORT jfloat JNICALL Java_pcl_pointxyz_getY
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointxyz_setY
	(JNIEnv *, jobject, jfloat);

	JNIEXPORT jfloat JNICALL Java_pcl_pointxyz_getZ
	(JNIEnv *, jobject);

	JNIEXPORT void JNICALL Java_pcl_pointxyz_setZ
	(JNIEnv *, jobject, jfloat);

#ifdef __cplusplus
}
#endif
#endif
