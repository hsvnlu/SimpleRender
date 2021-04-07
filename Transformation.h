#pragma once
#include<Eigen/Dense>
#include<vector>
#define PI 3.1415926
using namespace Eigen;

class Transformation {
public:
	Transformation() {};

	void setViewMatrix(const Vector3f &eyePos, const Vector3f &eyeLookat, const Vector3f &upOrient);
	void setProjectionMatrix(float eye_fov, float aspect_ratio, float zNear, float zFar);
	static Matrix4f setTranslation(const Vector3f& t) {
		Matrix4f translate = Matrix4f::Identity();
		translate << 1.f, 0.f, 0.f, t.x(),
		0.f, 1.f, 0.f, t.y(),
		0.f, 0.f, 1.f, t.z(),
		0.f, 0.f, 0.f, 1.f;
		return translate;
	}
	static Matrix4f setRotation(float angel, const Vector3f& axis) {
		angel *= PI / 180.f;
		Matrix4f outPut = Matrix4f::Identity();
		float cos_a = cos(angel), sin_a = sin(angel);
		outPut(0, 0) = cos_a + (1 - cos_a) * axis.x() * axis.x();
		outPut(0, 1) = -sin_a * axis.z() + (1 - cos_a) * axis.x() * axis.y();
		outPut(0, 2) = sin_a * axis.y() + (1 - cos_a) * axis.x() * axis.z();
		outPut(1, 0) = sin_a * axis.z() + (1 - cos_a) * axis.x() * axis.y();
		outPut(1, 1) = cos_a + (1 - cos_a) * axis.y() * axis.y();
		outPut(1, 2) = -sin_a * axis.x() + (1 - cos_a) * axis.y() * axis.z();
		outPut(2, 0) = -sin_a * axis.y() + (1 - cos_a) * axis.z() * axis.x();
		outPut(2, 1) = sin_a * axis.x() + (1 - cos_a) * axis.z() * axis.y();
		outPut(2, 2) = cos_a + (1 - cos_a) * axis.z() * axis.z();
		return outPut;
	}
	static Matrix4f setScalar(const Vector3f& s) {
		Matrix4f scalar;
		scalar << s.x(), 0.f, 0.f, 0.f,
			0.f, s.y(), 0.f, 0.f,
			0.f, 0.f, s.z(), 0.f,
			0.f, 0.f, 0.f, 1.f;
		return scalar;
	}
	//~Transformation() {};


	Matrix4f V = Matrix4f::Identity();
	Matrix4f P = Matrix4f::Identity();
};

