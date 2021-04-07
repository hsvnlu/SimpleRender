#include"Triangle.h"


void Triangle::setVertex(int ind, const Vector4f &position) {
	vertex[ind] = position;
}

void Triangle::setNormals(const std::array<Vector4f, 3> &normals)
{
	vertexNormal[0] = normals[0];
	vertexNormal[1] = normals[1];
	vertexNormal[2] = normals[2];
}

void Triangle::setNormal(int ind, Vector4f normal)
{
	vertexNormal[ind] = normal;
}

void Triangle::setTexCoords(const std::array<Vector2f, 3> &texCoords)
{
	texCoord[0] = texCoords[0];
	texCoord[1] = texCoords[1];
	texCoord[2] = texCoords[2];
}

void Triangle::setTexCoord(int ind, const Vector2f &uv)
{
	texCoord[ind] = uv;
}

