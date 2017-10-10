#pragma once

#include <GUILib/GLMesh.h>
#include <GUILib/OBJReader.h>
#include <process.h> 
#include <string>

using namespace std;

// Mesh Boolean Operation, the result mesh will be returned.
// boolean_type: "Union", "Intersect", "Minus", "XOR", "Resolve"

inline GLMesh* meshBooleanCork(GLMesh* mesh_A, GLMesh* mesh_B, const char* boolean_type)
{
	mesh_A->writeToOFF("../out/tmpMeshA.off");
	mesh_B->writeToOFF("../out/tmpMeshB.off");

	string cork_boolean_type;
	if (strcmp(boolean_type, "Union") == 0)
		cork_boolean_type = "-union";
	else if (strcmp(boolean_type, "Intersect") == 0)
		cork_boolean_type = "-isct";
	else if (strcmp(boolean_type, "Minus") == 0)
		cork_boolean_type = "-diff";
	else if (strcmp(boolean_type, "XOR") == 0)
		cork_boolean_type = "-xor";
	else if (strcmp(boolean_type, "Resolve") == 0)
		cork_boolean_type = "-resolve";

	int res = spawnl(P_WAIT, "../MeshUtilsPrograms/MeshBooleanCork.exe", "MeshBooleanCork.exe",
		cork_boolean_type.c_str(), "../out/tmpMeshA.off", "../out/tmpMeshB.off", "../out/BooleanRes.off", NULL);

	if (res != 0) {
		Logger::print("Mesh Boolean operation failed!\n");
		return NULL;
	}

	GLMesh* resMesh = new GLMesh();
	resMesh->loadFromOFF("../out/BooleanRes.off");
	return resMesh;
}

// Mesh Boolean Operation, the result mesh will be returned.
// boolean_type: "Union", "Intersect", "Minus", "XOR", "Resolve"

inline void meshBooleanIntrusiveCork(GLMesh*& mesh_A, GLMesh* mesh_B, const char* boolean_type)
{
	mesh_A->writeToOFF("../out/tmpMeshA.off");
	mesh_B->writeToOFF("../out/tmpMeshB.off");

	string cork_boolean_type;
	if (strcmp(boolean_type, "Union") == 0)
		cork_boolean_type = "-union";
	else if (strcmp(boolean_type, "Intersect") == 0)
		cork_boolean_type = "-isct";
	else if (strcmp(boolean_type, "Minus") == 0)
		cork_boolean_type = "-diff";
	else if (strcmp(boolean_type, "XOR") == 0)
		cork_boolean_type = "-xor";
	else if (strcmp(boolean_type, "Resolve") == 0)
		cork_boolean_type = "-resolve";

	int res = spawnl(P_WAIT, "../MeshUtilsPrograms/MeshBooleanCork.exe", "MeshBooleanCork.exe",
		cork_boolean_type.c_str(), "../out/tmpMeshA.off", "../out/tmpMeshB.off", "../out/BooleanRes.off", NULL);

	if (res != 0) {
		Logger::print("Mesh Boolean operation failed!\n");
		return;
	}

	delete mesh_A;
	mesh_A = new GLMesh();
	mesh_A->loadFromOFF("../out/BooleanRes.off");
}


// Mesh Boolean Operation, the result mesh will be returned.
// boolean_type: "Union", "Intersect", "Minus", "XOR", "Resolve"

inline GLMesh* meshBoolean(GLMesh* mesh_A, GLMesh* mesh_B, const char* boolean_type)
{
	mesh_A->writeTriangulatedMeshToObj("../out/tmpMeshA.obj");

	mesh_B->writeTriangulatedMeshToObj("../out/tmpMeshB.obj");

	int res = spawnl(P_WAIT, "../MeshUtilsPrograms/MeshBoolean.exe",
		"MeshBoolean.exe", "../out/tmpMeshA.obj", "../out/tmpMeshB.obj", "../out/BooleanRes.obj", boolean_type, NULL);

	if (res != 0) {
		Logger::print("Mesh Boolean operation failed!\n");
		return NULL;
	}

	return OBJReader::loadOBJFile("../out/BooleanRes.obj");
}

// Mesh Boolean Operation, it will modify mesh_A directly.
// boolean_type: "Union", "Intersect", "Minus", "XOR", "Resolve"
inline bool meshBooleanIntrusive(GLMesh*& mesh_A, GLMesh* mesh_B, const char* boolean_type)
{
	mesh_A->writeTriangulatedMeshToObj("../out/tmpMeshA.obj");

	mesh_B->writeTriangulatedMeshToObj("../out/tmpMeshB.obj");

	int res = spawnl(P_WAIT, "../MeshUtilsPrograms/MeshBoolean.exe",
		"MeshBoolean.exe", "../out/tmpMeshA.obj", "../out/tmpMeshB.obj", "../out/BooleanRes.obj", boolean_type, NULL);

	if (res != 0) {
		Logger::print("Mesh Boolean operation failed!\n");
		return false;
	}

	delete mesh_A;
	mesh_A = OBJReader::loadOBJFile("../out/BooleanRes.obj");

	return true;
}

inline GLMesh* meshBoolean(const char* mesh_A, const char* mesh_B, const char* boolean_type)
{
	int res = spawnl(P_WAIT, "../MeshUtilsPrograms/MeshBoolean.exe",
		"MeshBoolean.exe", mesh_A, mesh_B, "../out/BooleanRes.obj", boolean_type, NULL);

	if (res != 0) {
		Logger::print("Mesh Boolean operation failed!\n");
		return NULL;
	}

	return OBJReader::loadOBJFile("../out/BooleanRes.obj");
}