#pragma once
#include <Eigen/Dense>
#include<vector>
using namespace Eigen;

class Light
{
public:
	Light() {};
	Light(Vector4f pos, Vector3f intsty);

	//~Light();

	Vector4f position;
	Vector3f intensity;

};

inline Light::Light(Vector4f pos, Vector3f intsty)
{
	position = pos;
	intensity = intsty;
}