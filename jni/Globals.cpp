#include "Globals.h"


bool isShowRects = true;
bool isShowPoints = true;
bool isShowTexts = true;
bool isMultiScale = true;


PMesh* makeMesh(string path)
{
	PMesh::EdgeCost g_edgemethod = PMesh::QUADRICTRI;
	const char* charPath = path.c_str();
	Mesh* mesh = new Mesh(charPath);
	//vector<vertex>& vert = g_pMesh->_vlist;

	if (mesh)
	{
		mesh->Normalize(0.2f);// center mesh around the origin & shrink to fit
	}
	cout << "after normal" << endl;


	PMesh* pmesh = new PMesh(mesh, g_edgemethod);
	return pmesh;
}

Model::Model(int index)
{
	if (index >= modelPathList.size())
	{
		return;
	}
	pmesh = makeMesh(modelPathList[index]);
	edgeNum = pmesh->numEdgeCollapses();
}

Model::Model(string path)
{
	pmesh = makeMesh(path);
	edgeNum = pmesh->numEdgeCollapses();
}

Model::Model()
{
	//meshIndex = 0;
}

int Model::meshEdgeNum()
{
	if (pmesh)
	{
		return pmesh->numEdgeCollapses();
	}
	return 0;
}

int Model::getActiveNum()
{
	int oldNum = 0;
	if (!pmesh)
	{
		return 0;
	}
	Mesh& m = pmesh->_newmesh;
	int num = 0;
	for (int i = 0; i<m._plist.size(); i++)
	{
		if (m._plist[i].isActive()){
			num++;
		}
	}
	return num;
}

int Model::getEdgeNum(float distance)
{
	cout << "calc edge from distance" << endl;
	cout << distance;
	if (distance <= MIN_DISTANCE)
	{
		return (float)MIN_EDGE*(float)meshEdgeNum();
	}
	else if (distance >= MAX_DISTANCE)
	{
		return (float)MAX_EDGE*(float)meshEdgeNum();
	}
	else{
		float persent = (distance - MIN_DISTANCE)*(MAX_EDGE - MIN_EDGE) / (MAX_DISTANCE - MIN_DISTANCE) + MIN_EDGE;
		cout << persent << endl;
		int edge = (float)meshEdgeNum()*persent;
		cout << edge << endl;
		return edge;
	}
}


vector<Model> kfmodels;


ARPipeline pipeline;
ARDrawingContext drawingCtx;




CameraCalibration calibration(640.0f, 640.0f, 320.0f, 240.0f);


//Mesh *g_pMesh = NULL; // not necessary, but a nice CYA habit
//PMesh *g_pProgMesh = NULL;


GLuint *gTexture = 0;

ARDrawing drawing;
ARError arerror;
AREngine engine;



//models
string imagePath;


vector<Point2f> pointList;

vector<string> modelPathList;

vector<PMesh* > pmeshList;


vector<Eye> eyes;


void changeEdgeNum(Model& model, int newNum)
{
	bool ret = true;
	PMesh* pMesh = model.pmesh;
	if (pMesh == NULL)
	{
		return;
	}
	int oldNum = model.edgeNum;
	//int oldNum = model.getActiveNum();
	/*Mesh& m = pMesh->_newmesh;
	for (int i = 0; i<m._plist.size(); i++)
	{
		if (m._plist[i].isActive()){
			oldNum++;
		}
	}*/
	if (oldNum == newNum)
	{
		return;
	}
	else if (oldNum > newNum)//remove
	{
		for (int i = 0; i < oldNum - newNum; i++)
		{
			ret = pMesh->collapseEdge();
		}
	}
	else// add
	{
		for (int i = 0; i < newNum - oldNum; i++)
		{
			ret = pMesh->splitVertex();
		}
	}
	cout << "change edge num: " << endl;
	cout << oldNum << "  " << newNum << endl;
	int n = model.getActiveNum();
	cout << n << endl;
	model.edgeNum = newNum;
}


