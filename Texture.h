#pragma once
#include<opencv2/opencv.hpp>

class Texture
{
private:
    cv::Mat image_data;

public:
    Texture() = default;
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getTexColor(float u, float v) const
    {
        if (u < 0.f) u = 0.f;
        if (v < 0.f) v = 0.f;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};