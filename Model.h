#pragma once
#include"Triangle.h"
#include"Texture.h"
#include<Eigen/Dense>
#include<string>



class Model
{
public:
	Model();
	Model(const std::string& objPath, const std::string& texPath);
	//~Model();
	void setModelMatrix(const Matrix4f& Translation, const Matrix4f& Rotation, const Matrix4f& Scalar);

	Texture texture;
	std::vector<Triangle*> TriangleList;
	Eigen::Matrix4f M;
};