#include"Transformation.h"
#include<iostream>

void Transformation::setViewMatrix(const Vector3f& eyePos, const Vector3f& eyeLookat, const Vector3f& upOrient)
{
	Vector3f direction = (eyeLookat - eyePos).normalized();
	Vector3f for_x = direction.cross(upOrient).normalized();
	Vector3f for_y = for_x.cross(direction);
	V << for_x.x(), for_x.y(), for_x.z(), -eyePos.x(),
		for_y.x(), for_y.y(), for_y.z(), -eyePos.y(),
		-direction.x(), -direction.y(), -direction.z(), -eyePos.z(),
		0.f, 0.f, 0.f, 1.f;
	/*for (int i = 0; i < 4; ++i) {
		for(int j = 0; j < 4; ++j){
		std::cout << V(i, j) << ' ';
		}
	std::cout << '\n';
	}*/
}

void Transformation::setProjectionMatrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	eye_fov = eye_fov * PI / 180.f;
	P = Matrix4f::Identity();
	float tanfov_2 = tan(eye_fov / 2);
	P(0, 0) = -1 / (tanfov_2 * aspect_ratio);
	P(1, 1) = -1 / (tanfov_2);
	P(2, 2) = (zNear + zFar) / (zNear - zFar);
	P(2, 3) = 2 * zNear * zFar / (zFar - zNear);
	P(3, 2) = 1.f;
	P(3, 3) = 0.f;
}