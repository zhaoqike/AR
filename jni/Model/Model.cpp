#include <jni.h>
#include "mesh.h"
#include "pmesh.h"
#include <android/log.h>

#define  LOG_TAG    "libgljni"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

extern PMesh *g_pProgMesh;
extern Mesh *g_pMesh;



//Mesh *g_pMesh = NULL; // not necessary, but a nice CYA habit
//PMesh *g_pProgMesh = NULL;
//Mesh mesh;




extern "C" {
JNIEXPORT void JNICALL Java_com_example_model_ModelNativeLib_print(JNIEnv*, jobject);
}

JNIEXPORT void JNICALL Java_com_example_model_ModelNativeLib_print(JNIEnv*, jobject)
{
	LOGE("print");
	PMesh::EdgeCost g_edgemethod = PMesh::QUADRICTRI;
	g_pMesh = new Mesh("/sdcard/cow.ply");
	vector<vertex>& vert = g_pMesh->_vlist;

	if (g_pMesh) g_pMesh->Normalize();// center mesh around the origin & shrink to fit
	cout<<"after normal"<<endl;


	g_pProgMesh = new PMesh(g_pMesh, g_edgemethod );
	//g_pProgMesh->print();
}


