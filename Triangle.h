#ifndef OBJ_TRIANGLE_H
#define OBJ_TRIANGLE_H

#pragma once
#include<eigen/Dense>
#include<opencv2/opencv.hpp>
#include<algorithm>
#include<array>
using namespace Eigen;

class Triangle {
public:
	void setVertex(int ind, const Vector4f &position);
	void setNormals(const std::array<Vector4f, 3> &normals);
	void setNormal(int ind, Vector4f normal);
	void setTexCoords(const std::array<Vector2f, 3> &texCoords);
	void setTexCoord(int ind, const Vector2f &uv);

	Vector4f vertex[3];
	Vector4f vertexNormal[3];
	Vector2f texCoord[3];
	unsigned int index[3];
	
};
#endif