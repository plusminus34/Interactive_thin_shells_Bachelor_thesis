#include <RobotDesignerLib/RMCPin.h>
#include <GUILib/GLApplication.h>
#include <MathLib/Segment.h>

bool RMCPin::showCoordinate = false;

RMCPin::RMCPin(RMC* _rmc, int _id)
{
	rmc = _rmc;
	joint = NULL;
	idle = true;
	id = _id;
}

RMCPin::RMCPin(RMC* _rmc, const Transformation& _transformation, int _id)
{
	rmc = _rmc;
	transformation = _transformation;
	joint = NULL;
	idle = true;
	id = _id;
}

RMCPin::~RMCPin()
{
}

void RMCPin::detach()
{
	idle = true;
	joint = NULL;
}

void RMCPin::draw(const V3D& color)
{
	if (!idle) return;

	glPushMatrix();
	Transformation trans(rmc->state.orientation.getRotationMatrix(), rmc->state.position);
	trans *=  transformation;

	// TODO: put applyGLMatrixTransform outside of MatLib
	//trans.applyGLMatrixTransform();

	if (showCoordinate)
	{
		glColor3d(1.0, 1.0, 0.0);
		drawArrow(P3D(), P3D(0.01, 0, 0), 0.001, 12);
		glColor3d(0.0, 1.0, 1.0);
		drawArrow(P3D(), P3D(0, 0.01, 0), 0.001, 12);
		glColor3d(1.0, 0.0, 1.0);
		drawArrow(P3D(), P3D(0, 0, 0.01), 0.001, 12);
	}
	
	glDisable(GL_TEXTURE_2D);
	glColor3d(color[0], color[1], color[2]);
	drawSphere(P3D(), 0.0035, 12); //0.002
	glEnable(GL_TEXTURE_2D);

	glPopMatrix();
}

void RMCPin::loadFromFile(FILE* fp)
{
	char buffer[200];
	char keyword[50];

	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		Logger::print("%s ", keyword);

		// compatibleMap are taken care of by transformationMap
		/*if (strcmp(keyword, "Compatible") == 0)
		{
			string str = line + strlen(keyword);
			int index = str.find_first_not_of(' ');
			while (index != string::npos)
			{
				int sIndex = str.find_first_of(' ', index);
				if (sIndex == string::npos) {
					compatibleMap.insert(str.substr(index));
					break;
				}
				else
					compatibleMap.insert(str.substr(index, sIndex - index));

				index = str.find_first_not_of(' ', sIndex);
			}

			for (auto itr = compatibleMap.begin(); itr != compatibleMap.end(); itr++)
			{
				Logger::print("%s ", itr->c_str());
			}		
		}*/
		if (strcmp(keyword, "Name") == 0)
		{
			char content[200];
			int num = sscanf(line + strlen(keyword), "%s", content);
			name = content;
			Logger::print("%s ", name.c_str());
		}
		else if (strcmp(keyword, "FacePoint") == 0)
		{
			P3D p;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf",
				&p[0], &p[1], &p[2]);
			if (num < 3)
				throwError("Not enough transformation parameters!");

			face.vertices.push_back(p);
		}
		else if (strcmp(keyword, "FaceCenter") == 0)
		{
			P3D p;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf",
				&p[0], &p[1], &p[2]);
			if (num < 3)
				throwError("Not enough transformation parameters!");

			face.center = p;
		}
		else if (strcmp(keyword, "FaceNormal") == 0)
		{
			V3D n;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf",
				&n[0], &n[1], &n[2]);
			if (num < 3)
				throwError("Not enough transformation parameters!");

			face.normal = n;
		}
		else if (strcmp(keyword, "Transformation") == 0)
		{
			Matrix3x3& R = transformation.R;
			Vector3d& T = transformation.T;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&R(0, 0), &R(0, 1), &R(0, 2), &R(1, 0), &R(1, 1), &R(1, 2), &R(2, 0), &R(2, 1), &R(2, 2),
				&T[0], &T[1], &T[2]);
			if (num < 12)
				throwError("Not enough transformation parameters!");

			for (int i = 0; i < 9; i++)
				Logger::print("%lf ", R(i / 3, i % 3));
			for (int i = 0; i < 3; i++)
				Logger::print("%lf ", T[i]);
		}
		else if (strcmp(keyword, "AngleAxis") == 0)
		{
			Matrix3x3& R = transformation.R;
			V3D axis;
			double angle;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf %lf",
				&angle, &axis[0], &axis[1], &axis[2]);
			if (num < 4)
				throwError("Not enough transformation parameters!");

			R *= AngleAxisd(RAD(angle), axis).toRotationMatrix();

			for (int i = 0; i < 9; i++)
				Logger::print("%lf ", R(i / 3, i % 3));
		}
		else if (strcmp(keyword, "Translation") == 0)
		{
			Vector3d& T = transformation.T;
			V3D tmpT;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf",
				&tmpT[0], &tmpT[1], &tmpT[2]);
			if (num < 3)
				throwError("Not enough transformation parameters!");

			T += tmpT;

			for (int i = 0; i < 3; i++)
				Logger::print("%lf ", T[i]);
		}
		else if (strcmp(keyword, "Horn") == 0)
		{
			type = HORN_PIN;
		}
		else if (strcmp(keyword, "EndPin") == 0)
		{
			break;
		}
		Logger::print("\n");
	}
}

bool RMCPin::isPicked(Ray& ray)
{
	Transformation trans(rmc->state.orientation.getRotationMatrix(), rmc->state.position);
	trans *= transformation;
	
	if (ray.getDistanceToPoint(trans.transform(P3D())) < PICK_THRESHOLD)
	{
		return true;
	}

	return false;
}

bool RMCPin::isCompatible(RMCPin* pin)
{
	return compatibleMap.count(pin->name) > 0 && pin->compatibleMap.count(name);
}

RMCPin* RMCPin::getConnectedPin()
{
	if (idle) return NULL;

	return this == joint->parentPin ? joint->childPin : joint->parentPin;
}
