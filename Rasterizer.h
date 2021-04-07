#pragma once
#include<iostream>
#include<vector>
#include<Eigen/Dense>
#include"Triangle.h"
#include"Light.h"
#include"Texture.h"
#include"Model.h"
#include"Transformation.h"
using namespace Eigen;

class Rasterizer {
public:
	Rasterizer(unsigned int w, unsigned int h) { 
		width = w;
		height = h;
		depth_buffer = std::vector<float>(width * height, -std::numeric_limits<float>::infinity());
		frame_buffer = std::vector<Vector3f>(width * height, Vector3f(0.0f, 0.0f, 0.0f));
	}
	void rendering(const Model model, const std::vector<Light>& lights, const Transformation& trans);
	void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos);
	size_t getBufferIndex(float x, float y) const;



	unsigned int width;
	unsigned int height;
	std::vector<float> depth_buffer;
	std::vector<Vector3f> frame_buffer;

};
