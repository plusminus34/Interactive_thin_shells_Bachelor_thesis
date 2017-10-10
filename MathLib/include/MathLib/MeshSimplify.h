#pragma once

#include <GUILib/GLMesh.h>
#include <GUILib/OBJReader.h>
#include <process.h> 
#include <string>

using namespace std;

// Mesh Simplification, return a simplified mesh with desired ratio.

inline GLMesh* meshSimplify(GLMesh* mesh, double ratio = 0.5, double aggressiveness = 7.0)
{
	FILE* fp = fopen("../out/tmpMeshC.obj", "w+");
	mesh->renderToObjFile(fp, 0, Quaternion(), P3D());
	fclose(fp);

	int res = spawnl(P_WAIT, "../MeshUtilsPrograms/MeshSimplify.exe", "MeshSimplify.exe",
		"../out/tmpMeshC.obj", "../out/SimpRes.obj", to_string(ratio).c_str(), to_string(aggressiveness).c_str(), NULL);

	if (res != 0) {
		Logger::print("Mesh Simplication failed!\n");
		return NULL;
	}

	return OBJReader::loadOBJFile("../out/SimpRes.obj");
}
