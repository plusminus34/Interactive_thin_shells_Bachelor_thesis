#include <glad/glad.h>

#include <GUILib/GLApplication.h>
#include <GUILib/GLIncludes.h>
#include <GUILib/GLMesh.h>
#include <MathLib/V3D.h>
#include <map>
#include <GUILib/GLUtils.h>
#include <Utils/Utils.h>
#include <GUILib/GLTrackingCamera.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLTexture.h>
#include <GUILib/GLShaderMaterial.h>
#include <GUILib/GLIncludes.h>
#include <GUILib/GLWindow.h>
#include <Utils/Logger.h>
#include <Utils/Timer.h>
#include <GUILib/GLWindow2D.h>
#include <GUILib/GLConsole.h>
#include <GUILib/GLCamera.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GlobalMouseState.h>

/**
	this is the default constructor
*/
GLMesh::GLMesh(void){
	vertexList = DynamicArray<double>();
	normalList = DynamicArray<double>();
	texCoordList = DynamicArray<double>();
	tangentList = DynamicArray<double>();
	DynamicArray<SharedVertexInfo*> sharedVertices;
	polygons = new GLPolyCategory();
	useNormals = true;
	nrPolys = 0;
	vertexCount = 0;
}

GLMesh* GLMesh::clone(){
	GLMesh* mesh = new GLMesh();
	mesh->vertexList = this->vertexList;
	mesh->normalList = this->normalList;
	mesh->texCoordList = this->texCoordList;
	mesh->tangentList = this->tangentList;
	mesh->useNormals = this->useNormals;
	mesh->nrPolys = this->nrPolys; mesh->vertexCount = this->vertexCount;
	mesh->triangles = this->triangles;
	for(uint i=0;i<this->polygons->categories.size();i++)
		mesh->polygons->categories.push_back(new GLPolyIndexList(*(this->polygons->categories[i])));
	for(uint i=0;i<sharedVertices.size();i++)
		if(sharedVertices[i]==NULL)
			mesh->sharedVertices.push_back(NULL);
		else
			mesh->sharedVertices.push_back(new SharedVertexInfo(*sharedVertices[i]));
	mesh->material = material;
	mesh->bbox = bbox;
	mesh->path = path;
	return mesh;
}

/**
	this is the destructor
*/
GLMesh::~GLMesh(void){
	for (int i=0;i<vertexCount;i++)
		if (sharedVertices[i]!=NULL)
			delete sharedVertices[i];
	delete polygons;
}

/**
	this method is used to add a new vertex to the mesh
*/
void GLMesh::addVertex(const P3D &coords){
	vertexList.push_back(coords[0]);
	vertexList.push_back(coords[1]);
	vertexList.push_back(coords[2]);

	//also make sure the normal/text coordinates have the same size...
	texCoordList.push_back(0.0);
	texCoordList.push_back(0.0);
	texCoordList.push_back(0.0);
	normalList.push_back(0.0);
	normalList.push_back(0.0);
	normalList.push_back(0.0);
	tangentList.push_back(0.0);
	tangentList.push_back(0.0);
	tangentList.push_back(0.0);
	sharedVertices.push_back(NULL);
	vertexCount++;
}

/**
	this method also adds texture coordinates for the vertex that is to be added.
*/
void GLMesh::addVertex(const P3D &coords, const P3D &texCoords){
	vertexList.push_back(coords[0]);
	vertexList.push_back(coords[1]);
	vertexList.push_back(coords[2]);

	//also make sure the normal/text coordinates have the same size...
	texCoordList.push_back(texCoords[0]);
	texCoordList.push_back(texCoords[1]);
	texCoordList.push_back(texCoords[2]);
	normalList.push_back(0.0);
	normalList.push_back(0.0);
	normalList.push_back(0.0);
	tangentList.push_back(0.0);
	tangentList.push_back(0.0);
	tangentList.push_back(0.0);
	sharedVertices.push_back(NULL);
	vertexCount++;
}

void GLMesh::addPrism(std::vector<P3D> &firstPoints, std::vector<P3D> &secondPoints, bool invertNormals){

	if(invertNormals){
		std::vector<P3D> aux;

		aux.assign(firstPoints.begin(),firstPoints.end());
		firstPoints.assign(secondPoints.begin(),secondPoints.end());
		secondPoints.assign(aux.begin(),aux.end());
	}

	int originalPoints = getVertexCount();

	uint nPoints = (uint)firstPoints.size();
	double oneOverNPoints = 1.0/(double)nPoints;

	P3D firstCentroid, secondCentroid;
	for(uint i = 0; i < nPoints; i++){
		firstCentroid += firstPoints[i];
		secondCentroid += secondPoints[i];
	}

	firstCentroid *= oneOverNPoints;
	secondCentroid *= oneOverNPoints;

	addVertex(firstCentroid);
	addVertex(secondCentroid);

	int midPoint1Idx = originalPoints;
	int midPoint2Idx = originalPoints+1;

	int firstPoint1 = getVertexCount();
	for(uint i = 0; i < nPoints; i++){
		addVertex(firstPoints[i]);
	}

	int firstPoint2 = getVertexCount();
	for(uint i = 0; i < nPoints; i++){
		addVertex(secondPoints[i]);
	}

	int currentIndex1, nextIndex1;
	int currentIndex2, nextIndex2;
	for(uint i = 0; i < nPoints; i++){
		currentIndex1 = firstPoint1+i;
		nextIndex1 = currentIndex1+1;
		if(i == nPoints-1)
			nextIndex1 = firstPoint1;
		GLIndexedTriangle tri = GLIndexedTriangle(currentIndex1,nextIndex1,midPoint1Idx,true);
		addPoly(tri);

		currentIndex2 = firstPoint2+i;
		nextIndex2 = currentIndex2+1;
		if(i == nPoints-1)
			nextIndex2 = firstPoint2;
		tri = GLIndexedTriangle(currentIndex2,nextIndex2,midPoint2Idx);
		addPoly(tri);

		GLIndexedQuad quad = GLIndexedQuad(currentIndex1,currentIndex2,nextIndex2,nextIndex1,true);
		addPoly(quad);
	}
}

/**
	this method add a cylinder to the current mesh.
*/
void GLMesh::addCylinder(const P3D &p1, const P3D &p2, double radius, int resolution){
	int originalPoints = getVertexCount();

	double rotAngle = 2.0*PI/(double)resolution;
	V3D rotAxis = V3D(p1,p2).unit();
	V3D v1,v2,disp;
	rotAxis.getOrthogonalVectors(v1,v2);

	for (int i=0;i<resolution;i++){
		double angle = rotAngle * i;
		
		disp = v1;
		disp = disp.rotate(angle,rotAxis)*radius;
		addVertex(p1+disp);
		addVertex(p2+disp);
	}

	for (int i=0; i<resolution;i++){
		int nextIndex = i+1;
		if (nextIndex == resolution) nextIndex = 0;
		GLIndexedPoly poly;
		poly.addVertexIndex(2*nextIndex+originalPoints);poly.addVertexIndex(2*nextIndex+1+originalPoints);poly.addVertexIndex(2*i+1+originalPoints);poly.addVertexIndex(2*i+originalPoints);
		addPoly(poly);
	}

	int middleVertexIndex = (int)getVertexCount()-originalPoints;
	P3D c1 = p1;
	P3D c2 = p2;
	addVertex(c1);
	addVertex(c2);

	for (int i=0; i<resolution; i++){
		int nextIndex = i+1;
		if (nextIndex == resolution) nextIndex = 0;
		GLIndexedPoly poly;
		poly.addVertexIndex(middleVertexIndex+originalPoints);poly.addVertexIndex(2*nextIndex+originalPoints);poly.addVertexIndex(2*i+originalPoints);
		addPoly(poly);
		poly.clear();
		poly.addVertexIndex(2*nextIndex+1+originalPoints);poly.addVertexIndex(middleVertexIndex+1+originalPoints);poly.addVertexIndex(2*i+1+originalPoints);
		addPoly(poly);
	}
}

void GLMesh::addCone(const P3D &basePoint, const P3D &topPoint, double radius, int resolution){
	int originalPoints = getVertexCount();

	double rotAngle = 2.0*PI/(double)resolution;
	V3D rotAxis = V3D(basePoint,topPoint).unit();
	V3D v1,v2,disp;
	rotAxis.getOrthogonalVectors(v1,v2);

	P3D c = basePoint, tp = topPoint;
	addVertex(c);
	addVertex(tp);
	int basePointIndex = originalPoints;
	int topPointIndex = originalPoints+1;

	for (int i=0;i<resolution;i++){
		double angle = rotAngle * i;
		
		disp = v1;
		disp = disp.rotate(angle,rotAxis)*radius;
		addVertex(c+disp);
	}

	int firstRadiusIndex = basePointIndex+2;
	for (int i=0; i<resolution;i++){
		int currentIndex = firstRadiusIndex+i;
		int nextIndex = currentIndex+1;
		if (i == resolution-1) nextIndex = firstRadiusIndex;
		GLIndexedTriangle triangle = GLIndexedTriangle(currentIndex,nextIndex,topPointIndex);
		addPoly(triangle);
	}

	for (int i=0; i<resolution; i++){
		int currentIndex = firstRadiusIndex+i;
		int nextIndex = currentIndex+1;
		if (i == resolution-1) nextIndex = firstRadiusIndex;
		GLIndexedTriangle triangle = GLIndexedTriangle(currentIndex,nextIndex,basePointIndex,true);
		addPoly(triangle);
	}
}

void GLMesh::addCube(const P3D &baseCorner, const P3D &topCorner)
{
	int startVIndex = getVertexCount();

	addVertex(baseCorner);
	addVertex(P3D(baseCorner[0], topCorner[1], baseCorner[2]));
	addVertex(P3D(baseCorner[0], topCorner[1], topCorner[2]));
	addVertex(P3D(baseCorner[0], baseCorner[1], topCorner[2]));
	
	addVertex(P3D(topCorner[0], baseCorner[1], baseCorner[2]));
	addVertex(P3D(topCorner[0], topCorner[1], baseCorner[2]));
	addVertex(P3D(topCorner[0], topCorner[1], topCorner[2]));
	addVertex(P3D(topCorner[0], baseCorner[1], topCorner[2]));

    GLIndexedTriangle tr(startVIndex, startVIndex + 2, startVIndex + 1);
    addPoly(tr);
	addPoly(GLIndexedTriangle(startVIndex, startVIndex + 3, startVIndex + 2));

	addPoly(GLIndexedTriangle(startVIndex + 4, startVIndex + 5, startVIndex + 6));
	addPoly(GLIndexedTriangle(startVIndex + 4, startVIndex + 6, startVIndex + 7));

	addPoly(GLIndexedTriangle(startVIndex, startVIndex + 1, startVIndex + 5));
	addPoly(GLIndexedTriangle(startVIndex, startVIndex + 5, startVIndex + 4));

	addPoly(GLIndexedTriangle(startVIndex + 3, startVIndex + 6, startVIndex + 2));
	addPoly(GLIndexedTriangle(startVIndex + 3, startVIndex + 7, startVIndex + 6));

	addPoly(GLIndexedTriangle(startVIndex + 1, startVIndex + 2, startVIndex + 6));
	addPoly(GLIndexedTriangle(startVIndex + 1, startVIndex + 6, startVIndex + 5));

	addPoly(GLIndexedTriangle(startVIndex, startVIndex + 4, startVIndex + 7));
	addPoly(GLIndexedTriangle(startVIndex, startVIndex + 7, startVIndex + 3));

	computeNormals();
}

/**
	this method sets the coordinates for an existing vertex
*/
void GLMesh::setVertexCoordinates(int index, const P3D &coords){
	if (index>=0 && index<vertexCount){
		vertexList[3*index+0] = coords[0];
		vertexList[3*index+1] = coords[1];
		vertexList[3*index+2] = coords[2];
	}
}

/**
	This method sets the texture coordinates for an existing vertex.
*/
void GLMesh::setVertexTexCoordinates(int index, const P3D &texCoords){
	if (index>=0 && index<vertexCount){
		texCoordList[3*index+0] = texCoords[0];
		texCoordList[3*index+1] = texCoords[1];
		texCoordList[3*index+2] = texCoords[2];
	}
}

/**
	This method sets the normal for an existing vertex.
*/
void GLMesh::setVertexNormal(int index, const V3D &normal)
{
	if (index>=0 && index<vertexCount){
		useNormals = true;
		normalList[3*index+0] = normal[0];
		normalList[3*index+1] = normal[1];
		normalList[3*index+2] = normal[2];
	}
}

bool GLMesh::vertexIsInUse(int vIndex){
	for (uint i=0;i<polygons->categories.size();i++){
		for (uint j=0;j<polygons->categories[i]->indexList.size();j++)
			if (polygons->categories[i]->indexList[j] == vIndex)
				return true;
	}
	return false;
}

/**
	This is the method that adds new polygons to the mesh. The polygons have to be populated by the class that reads in the
	mesh from a file.
*/
void GLMesh::addPoly(const GLIndexedPoly &p){

    //now we must add the neighbour's info to each vertex in this poly
	for (uint i=0;i<p.indexes.size();i++){
		if (sharedVertices[p.indexes[i]] == NULL)
			sharedVertices[p.indexes[i]] = new SharedVertexInfo();
		int index, n1index, n2index;
		index = p.indexes[i];
		n1index = p.indexes[(i+1)%p.indexes.size()];
		n2index = p.indexes[(i-1+p.indexes.size())%p.indexes.size()];
		if (index<0 || index >= vertexCount ||n1index<0 || n1index >= vertexCount ||n2index<0 || n2index >= vertexCount)
			return;
		sharedVertices[index]->addVertexInfo(n1index, n2index);
	}
	polygons->addPoly(p);
	nrPolys++;
	//printf("%d ", p.indexes.size());
	//keep track of the individual triangles...
	if (p.indexes.size() == 3)
		triangles.emplace_back(p.indexes[0], p.indexes[1], p.indexes[2]);

	if (p.indexes.size() == 4){
		triangles.emplace_back(p.indexes[0], p.indexes[1], p.indexes[2]);
		triangles.emplace_back(p.indexes[0], p.indexes[2], p.indexes[3]);
	}

	if (p.indexes.size() > 4)
	{
		for (int i = 1; i < (int)p.indexes.size() - 1; i++)
		{
			triangles.emplace_back(p.indexes[0], p.indexes[i], p.indexes[i+1]);
		}
    }
}

void GLMesh::addPoly(const GLIndexedPoly &p, bool duplicateVertices)
{
    GLIndexedPoly pNew = p;
    if (duplicateVertices){
        //todo: should probably check to see if this vertex is used first...
        for (uint i=0;i<p.indexes.size();i++){
            if (vertexIsInUse(p.indexes[i])){
                addVertex(getVertex(p.indexes[i]));
                pNew.indexes[i] = getVertexCount()-1;
            }
        }
    }

    addPoly(pNew);
}

void GLMesh::addSphere(const P3D &center, double radius, int resolution){
	GLIndexedTriangle triangle = GLIndexedTriangle(0,0,0);
	GLIndexedQuad quad = GLIndexedQuad(0,0,0,0);
	
	int initVertexCount = getVertexCount();
	int nVerticalPoints = resolution;
	int nHorizontalPoints = 2+(resolution-2)*2;

	addVertex(center+V3D(0.0,0.0,radius));
	addVertex(center+V3D(0.0,0.0,-radius));

	double sinTheta, cosTheta, sinPhi, cosPhi;
	double theta = 0.0, phi = 0.0;
	double dPhi = PI/(double)(nVerticalPoints-1);
	double dTheta = dPhi;
	
	for(int h = 0; h < nHorizontalPoints; h++)
	{
		theta = dTheta*h;
		cosTheta = cos(theta);
		sinTheta = sin(theta);
		for(int v = 1; v < nVerticalPoints-1; v++)
		{
			phi = v*dPhi;
			cosPhi = cos(phi);
			sinPhi = sin(phi);
			addVertex(center+V3D(radius*sinPhi*cosTheta,radius*sinPhi*sinTheta,radius*cosPhi));
		}
	}

	int idx1, idx2, idx3, idx4;
	for(int h = 0; h < nHorizontalPoints; h++)
	{
		for(int v = 0; v < nVerticalPoints-3; v++)
		{
			idx1 = initVertexCount+2+h*(nVerticalPoints-2)+v;
			idx2 = initVertexCount+2+h*(nVerticalPoints-2)+v+1;
			if(h == nHorizontalPoints-1)
			{
				idx3 = initVertexCount+2+v+1;
				idx4 = initVertexCount+2+v;
			}
			else
			{
				idx3 = initVertexCount+2+(h+1)*(nVerticalPoints-2)+v+1;
				idx4 = initVertexCount+2+(h+1)*(nVerticalPoints-2)+v;
			}
			quad = GLIndexedQuad(idx1,idx2,idx3,idx4);
			addPoly(quad);
		}
	}

	for(int h = 0; h < nHorizontalPoints; h++)
	{
		idx1 = initVertexCount;
		idx2 = initVertexCount+2+h*(nVerticalPoints-2);
		if(h == nHorizontalPoints-1)
			idx3 = initVertexCount+2;
		else
			idx3 = initVertexCount+2+(h+1)*(nVerticalPoints-2);

		triangle = GLIndexedTriangle(idx1,idx2,idx3);
		addPoly(triangle);


		idx1 += 1;
		idx2 += nVerticalPoints-3;
		idx3 += nVerticalPoints-3;

		triangle = GLIndexedTriangle(idx1,idx2,idx3,true);
		addPoly(triangle);
	}
}

/**
	This method is used to compute the normals. The modifier passed in as a parameter should be either 1 or -1, and it
	should indicate if the polygons in this mesh have their vertices expressed in clockwise or anticlockwise order.
*/
void GLMesh::computeNormals(double modifier){
/*	Replace this... store the indexes of the two neighbours, instead of storing the way to retrieve them... 
		makes the code simpler*/
	useNormals = true;
	for (int i=0;i<vertexCount;i++){
		int nrNormals = 0;
		V3D result = V3D(0,0,0);
		if (sharedVertices[i] == NULL)
			continue;
		for (uint j=0;j<sharedVertices[i]->vertexInstances.size();j++){
			VertexNeighbourInfo tempVertexInfo = sharedVertices[i]->vertexInstances[j];
			//now look in that list, and retrieve the indexes of the current vertex, and its neighbours
			int n1Index = tempVertexInfo.n1Index;
			int n2Index = tempVertexInfo.n2Index;

			//now look in the vertex list, and construct the normal vector (i is the current index...)
			V3D v1 = V3D(P3D(vertexList[3*i+0],vertexList[3*i+1],vertexList[3*i+2]),
								P3D(vertexList[3*n1Index+0],vertexList[3*n1Index+1],vertexList[3*n1Index+2]));

			V3D v2 = V3D(P3D(vertexList[3*n2Index+0],vertexList[3*n2Index+1],vertexList[3*n2Index+2]),
								P3D(vertexList[3*i+0],vertexList[3*i+1],vertexList[3*i+2]));
			result+=((v2.cross(v1))*modifier).toUnit();
			nrNormals++;
		}
		//and finally, compute the resulting normal vector and store it...
		if (nrNormals>0)
			(result/=nrNormals);
		result.toUnit();
		normalList[3*i+0] = result[0];
		normalList[3*i+1] = result[1];
		normalList[3*i+2] = result[2];
	}
}

/**
	This method is used to compute the tangents from the UVs and vertex coordinates.
*/
void GLMesh::computeTangents(){
	// NOTE: We assume there is exactly one UV per vertex. Along seams, vertices are to be duplicated.
	for (int i=0; i<vertexCount; i++){
		V3D result = V3D(0,0,0);

		if (sharedVertices[i] == NULL)
			continue;

		for (uint j=0; j<sharedVertices[i]->vertexInstances.size(); j++){
			const VertexNeighbourInfo &tempVertexInfo = sharedVertices[i]->vertexInstances[j];

			//now look in that list, and retrieve the indexes of the current vertex, and its neighbours
			int n1Index = tempVertexInfo.n1Index;
			int n2Index = tempVertexInfo.n2Index;

			// Get the points and UVs
			V3D q1 = V3D(P3D(vertexList[3*i+0], vertexList[3*i+1], vertexList[3*i+2]),
								P3D(vertexList[3*n1Index+0], vertexList[3*n1Index+1], vertexList[3*n1Index+2]));
			V3D q2 = V3D(P3D(vertexList[3*i+0], vertexList[3*i+1], vertexList[3*i+2]),
								P3D(vertexList[3*n2Index+0], vertexList[3*n2Index+1], vertexList[3*n2Index+2]));
			
			V3D s1 = V3D(P3D(texCoordList[3*i+0], texCoordList[3*i+1], texCoordList[3*i+2]),
								P3D(texCoordList[3*n1Index+0], texCoordList[3*n1Index+1], texCoordList[3*n1Index+2]));
			V3D s2 = V3D(P3D(texCoordList[3*i+0], texCoordList[3*i+1], texCoordList[3*i+2]),
								P3D(texCoordList[3*n2Index+0], texCoordList[3*n2Index+1], texCoordList[3*n2Index+2]));

			// We want the tangent T to point in the direction of the uv's 'u' coordinate
			// For that, we solve the system
			// [q1[0]  q1[1]  q1[2]]     [s1[0]  s1[1]]   [T[0]  T[1]  T[2]]
			// [q2[0]  q2[1]  q2[2]]  =  [s2[0]  s2[1]] * [B[0]  B[1]  B[2]]
			// Where 'B' is the bitangent, which we don't need since we can compute it in the vertex shader. Inverting the '(s,t)' matrix, we compute
			// [T[0]  T[1]  T[2]]  =  1 / (s1[0]*s2[1] - s2[0]*s1[1]) * [ s2[1] -s1[1]] * [q1[0]  q1[1]  q1[2]]
			//                                                                   [q2[0]  q2[1]  q2[2]]

			double r = 1.0 / (s1[0]*s2[1] - s2[0]*s1[1]);
			V3D T;
			T[0] = r * (s2[1] * q1[0] - s1[1] * q2[0]);
			T[1] = r * (s2[1] * q1[1] - s1[1] * q2[1]);
			T[2] = r * (s2[1] * q1[2] - s1[1] * q2[2]);

			result += T;
		}

		//and finally, compute the resulting tangent vector and store it...
		result.toUnit();
		tangentList[3*i+0] = result[0];
		tangentList[3*i+1] = result[1];
		tangentList[3*i+2] = result[2];
	}
}


/**
	This method prints out the normals of the model - for testing purposes.
*/
void GLMesh::drawNormals(){
	for (int i=0;i<vertexCount;i++){
		glBegin(GL_LINES);
			glVertex3dv(&(vertexList.front())+3*i);
			glVertex3d(vertexList[3*i+0]+normalList[3*i+0],vertexList[3*i+1]+normalList[3*i+1],vertexList[3*i+2]+normalList[3*i+2]);
		glEnd();
	}
}

/**
	draws all the polygons of the mesh
*/
void GLMesh::drawMeshElements() {
	for (uint i = 0;i < polygons->categories.size();i++) {
		GLPolyIndexList* tempIndexList = polygons->categories[i];
		if (tempIndexList->polyVertexCount == 3)
			glDrawElements(GL_TRIANGLES, (GLsizei)tempIndexList->indexList.size(), GL_UNSIGNED_INT, &(tempIndexList->indexList.front()));
		else if (tempIndexList->polyVertexCount == 4)
			glDrawElements(GL_QUADS, (GLsizei)tempIndexList->indexList.size(), GL_UNSIGNED_INT, &(tempIndexList->indexList.front()));
		else {
			//handle polys that are not triangles or quads...
			for (unsigned int j = 0;j < tempIndexList->indexList.size() / tempIndexList->polyVertexCount;j++)
				glDrawElements(GL_POLYGON, tempIndexList->polyVertexCount, GL_UNSIGNED_INT, &(tempIndexList->indexList.front()) + j*tempIndexList->polyVertexCount);
		}
	}
}

/**
	This method draws the model.
*/
void GLMesh::drawMesh() {
	material.apply();

	/* enable the vertex array list*/
	if (vertexList.size() > 0) {
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &(vertexList.front()));
	}

	if (useNormals && normalList.size () > 0) {
		glEnableClientState(GL_NORMAL_ARRAY);
		glNormalPointer(GL_DOUBLE, 0, &(normalList.front()));
	}

	if (material.hasTextureParam() && tangentList.size() > 0) {
		glClientActiveTexture(GL_TEXTURE1);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(3, GL_DOUBLE, 0, &(tangentList.front()));
	}

	//glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	/* enable the texture coordinates arrays */
	if (material.hasTextureParam() && texCoordList.size() > 0) {
		glClientActiveTexture(GL_TEXTURE0);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(3, GL_DOUBLE, 0, &(texCoordList.front()));
	}
	if (IS_EQUAL(material.a,1))
		drawMeshElements();
	else {
		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT);
		drawMeshElements();
		glCullFace(GL_BACK);
		drawMeshElements();
		glDisable(GL_CULL_FACE);
	}
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	
	if (material.hasTextureParam()) {
		glClientActiveTexture(GL_TEXTURE1);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	}

	if (material.hasTextureParam()) {
		glClientActiveTexture(GL_TEXTURE0);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	}

	material.end();
}

// Renders the mesh by appending it to the OBJ file that is passed in. The vertices are transformed using the provided orientation and translation	
// vertexIdxOffset indicates the index of the first vertex for this object, making it possible to render multiple meshes to the same OBJ file
// Returns the number of vertices written to the file
uint GLMesh::renderToObjFile(FILE* fp, uint vertexIdxOffset, const Quaternion& rotation, const P3D& translation){
	P3D vertex;
	uint nbVerts = (uint)vertexList.size()/3;
	for( uint i = 0; i < nbVerts; i++ ) {
		vertex = translation + rotation * V3D(vertexList[i * 3 + 0], vertexList[i * 3 + 1], vertexList[i * 3 + 2]);
        fprintf( fp, "v %lf %lf %lf\n", vertex[0], vertex[1], vertex[2]);
	}

	fprintf( fp, "\n" );

	// Render faces
	for ( uint i = 0; i<polygons->categories.size(); i++ ){
		GLPolyIndexList* tempIndexList = polygons->categories[i];
		uint vertsPerFace = tempIndexList->polyVertexCount;
		uint nbFaces = (uint)tempIndexList->indexList.size() / vertsPerFace;
		uint idx = 0;
		for ( uint j = 0; j<nbFaces; j++ ) {
			fprintf(fp, "f ");
			for (uint k = 0; k < vertsPerFace; k++) {
				// Vertex indices are 1-based
				fprintf(fp, "%d ", tempIndexList->indexList[idx+k] + vertexIdxOffset + 1);
			}
			fprintf(fp, "\n");
//			uint nbTriInFan = vertsPerFace - 2;
//			for ( uint k = 0; k < nbTriInFan; k++ ) {
//				// Vertex indices are 1-based
//				fprintf( fp, "f %d %d %d \n", tempIndexList->indexList[idx]+vertexIdxOffset+1,
//					tempIndexList->indexList[idx+1+k]+vertexIdxOffset+1,
//					tempIndexList->indexList[idx+2+k]+vertexIdxOffset+1 );
//			}
			idx += vertsPerFace;
		}
	}

	fprintf( fp, "\n\n" );

	return nbVerts;
}

void GLMesh::writeToOFF(const char* fName)
{
	FILE* fp = fopen(fName, "w+");

	P3D vertex;
	uint nbVerts = (uint)vertexList.size() / 3;
	uint nbFaces = 0;
	for (uint i = 0; i < polygons->categories.size(); i++) {
		GLPolyIndexList* tempIndexList = polygons->categories[i];
		uint vertsPerFace = tempIndexList->polyVertexCount;
		nbFaces += (uint)tempIndexList->indexList.size() / vertsPerFace;
	}
	fprintf(fp, "OFF\n");
	fprintf(fp, "%d %d 0\n", nbVerts, nbFaces);

	for (uint i = 0; i < nbVerts; i++) {
		P3D vertex(vertexList[i * 3 + 0], vertexList[i * 3 + 1], vertexList[i * 3 + 2]);
		fprintf(fp, "%lf %lf %lf\n", vertex[0], vertex[1], vertex[2]);
	}

	// Render faces
	for (uint i = 0; i < polygons->categories.size(); i++) {
		GLPolyIndexList* tempIndexList = polygons->categories[i];
		uint vertsPerFace = tempIndexList->polyVertexCount;
		uint nbFaces = (uint)tempIndexList->indexList.size() / vertsPerFace;
		uint idx = 0;
		for (uint j = 0; j < nbFaces; j++) {
			fprintf(fp, "%d ", vertsPerFace);
			for (uint k = 0; k < vertsPerFace; k++) {
				// Vertex indices are 0-based
				fprintf(fp, "%d ", tempIndexList->indexList[idx + k]);
			}
			fprintf(fp, "\n");
			idx += vertsPerFace;
		}
	}
	fclose(fp);
}

void GLMesh::loadFromOFF(const char* fName)
{
	FILE* fp = fopen(fName, "r");

	uint nbVerts;
	uint nbFaces;
	uint nbEdges;
	fscanf(fp, "OFF");
	fscanf(fp, "%d %d %d", &nbVerts, &nbFaces, &nbEdges);

	for (uint i = 0; i < nbVerts; i++) {
		P3D vertex;
		fscanf(fp, "%lf %lf %lf", &vertex[0], &vertex[1], &vertex[2]);
		addVertex(vertex);
	}

	for (uint i = 0; i < nbFaces; i++) {
		int tmp;
		int v1, v2, v3;
		fscanf(fp, "%d %d %d %d", &tmp, &v1, &v2, &v3);
		GLIndexedTriangle triIndex(v1, v2, v3);
		addPoly(triIndex);
	}
	fclose(fp);

	computeNormals();
}

void GLMesh::writeTriangulatedMeshToObj(const char* fName)
{
	FILE* fp = fopen(fName, "w+");

	P3D vertex;
	uint nbVerts = (uint)vertexList.size() / 3;
	uint nbFaces = (uint)getTriangleCount();

	for (uint i = 0; i < nbVerts; i++) {
		P3D vertex(vertexList[i * 3 + 0], vertexList[i * 3 + 1], vertexList[i * 3 + 2]);
		fprintf(fp, "v %lf %lf %lf\n", vertex[0], vertex[1], vertex[2]);
	}

	fprintf(fp, "\n");

	for (uint i = 0; i < nbFaces; i++)
	{
		GLIndexedTriangle tri = getTriangle(i);
		fprintf(fp, "f %d %d %d\n", tri.indexes[0] + 1, tri.indexes[1] + 1, tri.indexes[2] + 1);
	}

	fclose(fp);
}

GLMesh GLMesh::increaseMeshDensity()
{
    int splitScale = 2;
    for (;splitScale * splitScale * getPolyCount() < 8000; ++splitScale);
    int tmpSplitScale = splitScale;
    //splitScale = 5;

    int tot = 0;
    std::map<std::pair<std::pair<int, int>, int>, int> edgePointIndex;
    edgePointIndex.clear();
    GLMesh resultMesh;
    resultMesh.clear();

    for (int i = 0;i < getVertexCount();++i)
        resultMesh.addVertex(P3D(vertexList[i * 3 + 0], vertexList[i * 3 + 1], vertexList[i * 3 + 2]));
    for (int i = 0;i < getTriangleCount(); ++i)
    {
        GLIndexedTriangle nowTriangle = getTriangle(i);
        for (int j = 0;j < 3;++j)
        {
            int x = nowTriangle.indexes[j];
            int y = nowTriangle.indexes[(j + 1) % 3];
            if (x > y) std::swap(x, y);
            if (edgePointIndex.find(std::make_pair(std::make_pair(x, y), 1)) != edgePointIndex.end())
                continue;
            P3D vx(vertexList[x * 3 + 0], vertexList[x * 3 + 1], vertexList[x * 3 + 2]);
            P3D vy(vertexList[y * 3 + 0], vertexList[y * 3 + 1], vertexList[y * 3 + 2]);
            V3D delt = (vy - vx) / splitScale;
            P3D splitPoint = vx;
            for (int k = 1;k < splitScale;++k)
            {
                splitPoint = splitPoint + delt;
                edgePointIndex[std::make_pair(std::make_pair(x, y), k)] = resultMesh.getVertexCount();
                resultMesh.addVertex(splitPoint);
            }
        }
    }
    for (int i = 0;i < getTriangleCount();++i)
    {
        GLIndexedTriangle nowTriangle = getTriangle(i);

        int id[3];
        for (int j = 0;j < 3;++j)
            id[j] = nowTriangle.indexes[j];
        P3D p[3];
        for (int j = 0;j < 3;++j)
            p[j] = P3D(vertexList[id[j] * 3 + 0], vertexList[id[j] * 3 + 1], vertexList[id[j] * 3 + 2]);

        std::map<std::pair<int, int>, int> innerPointIndex;
        innerPointIndex.clear();
        // create indexes for every new points;
        // add vertexes of triangle
        innerPointIndex[std::make_pair(0, 0)] = id[0];
        innerPointIndex[std::make_pair(splitScale, 0)] = id[1];
        innerPointIndex[std::make_pair(splitScale, splitScale)] = id[2];
        // add points on edges
        {
            int idx = id[0];
            int idy = id[1];
            for (int k = 1;k < splitScale;++k)
                if (idx < idy)
                {
                    innerPointIndex[std::make_pair(k, 0)] = edgePointIndex[std::make_pair(std::make_pair(idx, idy), k)];
                }
                else
                {
                    innerPointIndex[std::make_pair(k, 0)] = edgePointIndex[std::make_pair(std::make_pair(idy, idx), splitScale - k)];
                }
        }
        {
            int idx = id[1];
            int idy = id[2];
            for (int k = 1;k < splitScale;++k)
                if (idx < idy)
                {
                    innerPointIndex[std::make_pair(splitScale, k)] = edgePointIndex[std::make_pair(std::make_pair(idx, idy), k)];
                }
                else
                {
                    innerPointIndex[std::make_pair(splitScale, k)] = edgePointIndex[std::make_pair(std::make_pair(idy, idx), splitScale - k)];
                }
        }
        {
            int idx = id[0];
            int idy = id[2];
            for (int k = 1;k < splitScale;++k)
                if (idx < idy)
                {
                    innerPointIndex[std::make_pair(k, k)] = edgePointIndex[std::make_pair(std::make_pair(idx, idy), k)];
                }
                else
                {
                    innerPointIndex[std::make_pair(k, k)] = edgePointIndex[std::make_pair(std::make_pair(idy, idx), splitScale - k)];
                }
        }
        // create vertexes inside the original triangle
        V3D v1 = (p[1] - p[0]) / (double)splitScale;
        V3D v2 = (p[2] - p[1]) / (double)splitScale;
        for (int delt1 = 1;delt1 < splitScale;++delt1)
            for (int delt2 = 1;delt2 < delt1;++delt2)
            {
                P3D newPoint = p[0] + v1 * delt1 + v2 * delt2;
                innerPointIndex[std::make_pair(delt1, delt2)] = resultMesh.getVertexCount();
                resultMesh.addVertex(newPoint);
            }
        
        // create new smaller triangles
        for (int delt1 = 0;delt1 < splitScale;++delt1)
            for (int delt2 = 0;delt2 <= delt1;++delt2)
            {
                int newId1 = innerPointIndex[std::make_pair(delt1, delt2)];
                int newId2 = innerPointIndex[std::make_pair(delt1 + 1, delt2)];
                int newId3 = innerPointIndex[std::make_pair(delt1 + 1, delt2 + 1)];
                int newId4 = innerPointIndex[std::make_pair(delt1, delt2 + 1)];
                resultMesh.addPoly(GLIndexedTriangle(newId1, newId2, newId3));
                if (delt2 < delt1)
                    resultMesh.addPoly(GLIndexedTriangle(newId1, newId3, newId4));
            }
    }
    
    return resultMesh;
}
/*
void GLMesh::applyTransformationMatrix(const TransformationMatrix &Rt){
	for(uint i = 0; i < vertexList.size(); i += 3){
		P3D p = P3D(vertexList[i],vertexList[i+1],vertexList[i+2]);
		p = Rt*p;
		vertexList[i] = p[0];
		vertexList[i+1] = p[1];
		vertexList[i+2] = p[2];
	}
}
*/

void GLMesh::clear(){
	vertexList.clear();
	normalList.clear();
	texCoordList.clear();
	tangentList.clear();
	for (int i = 0; i<vertexCount; i++)
		if (sharedVertices[i] != NULL)
			delete sharedVertices[i];
	sharedVertices.clear();
	delete polygons;
	polygons = new GLPolyCategory();
	triangles.clear();
	nrPolys = 0;
	vertexCount = 0;
}


GLMesh* GLMesh::getSubMesh(DynamicArray<int>& triList) {

	GLMesh* mesh = new GLMesh();

	std::map<int, int> vertexMap;
	int vIndex = 0;
	//Logger::print("T:%d P:%d \n", getTriangleCount(), getPolyCount());

	for (uint i = 0; i < triList.size(); i++)
	{
		GLIndexedTriangle triangle = getTriangle(triList[i]);
		DynamicArray<int>& indexes = triangle.indexes;
		if (indexes.size() != 3)
		{
			Logger::print("%d ", indexes.size());
		}
		for (uint j = 0; j < indexes.size(); j++)
		{
			if (vertexMap.count(indexes[j]) == 0)
			{		
				mesh->addVertex(getVertex(indexes[j]));
				mesh->setVertexNormal(vIndex, getNormal(indexes[j]));
				vertexMap[indexes[j]] = vIndex;
				indexes[j] = vIndex;
				vIndex++;
			}
			else {
				indexes[j] = vertexMap[indexes[j]];
			}
		}
		mesh->addPoly(triangle);
	}

	//mesh->computeNormals();

	return mesh;
}

bool GLMesh::getDistanceToRayOriginIfHit(const Ray& ray, double* distToOrigin) {
	
	double min_t = 1e10;
	
	if (!bbox.getDistanceToRayOriginIfHit(ray)) {
		if (distToOrigin)
			*distToOrigin = min_t;
		return false;
	}

	for (int i = 0; i < getTriangleCount(); i++) {
		GLIndexedTriangle tri = getTriangle(i);
		P3D p1 = getVertex(tri.indexes[0]);
		P3D p2 = getVertex(tri.indexes[1]);
		P3D p3 = getVertex(tri.indexes[2]);
		
		double t = ray.getDistanceToTriangle(p1, p2, p3);
		if (t >= 0) 
			min_t = std::min(t, min_t);
	}

	if (distToOrigin)
		*distToOrigin = min_t;

	return min_t < 1e10;
}

void GLMesh::append(GLMesh* mesh, bool flipNormal)
{
	uint vStartIndex = getVertexCount();

	for (int i = 0; i < mesh->getVertexCount(); i++)
	{
		addVertex(mesh->getVertex(i));
	}

	for (int i = 0; i < mesh->getTriangleCount(); i++)
	{
		GLIndexedTriangle triFace = mesh->getTriangle(i);
		addPoly(GLIndexedTriangle(triFace.indexes[0] + vStartIndex, triFace.indexes[1] + vStartIndex, triFace.indexes[2] + vStartIndex, flipNormal));
	}

	computeNormals();
}

GLMesh* GLMesh::getSplitVerticeMesh()
{
	GLMesh* nMesh = new GLMesh();
	for (int i = 0; i < getTriangleCount(); i++)
	{
		GLIndexedTriangle triFace = getTriangle(i);
		for (int j = 0; j < 3; j++)
			nMesh->addVertex(getVertex(triFace.indexes[j]));
		nMesh->addPoly(GLIndexedTriangle(3 * i, 3 * i + 1, 3 * i + 2));
	}

	nMesh->computeNormals();
	return nMesh;
}

void GLMesh::splitVertices()
{
	GLMesh* tMesh = clone();
	clear();

	for (int i = 0; i < tMesh->getTriangleCount(); i++)
	{
		GLIndexedTriangle triFace = tMesh->getTriangle(i);
		for (int j = 0; j < 3; j++)
			addVertex(tMesh->getVertex(triFace.indexes[j]));
		addPoly(GLIndexedTriangle(3 * i, 3 * i + 1, 3 * i + 2));
	}

	computeNormals();
	delete tMesh;
}
