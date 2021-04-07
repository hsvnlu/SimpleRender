#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<string>
#include"Rasterizer.h"
#include"Transformation.h"
#include"Light.h"
#include"Model.h"

using namespace Eigen;
#define SRC_WIDTH  1000.f
#define SRC_HEIGHT  1000.f

int main() {
    //载入模型
    std::string objPath = "diablo3_pose.obj";
    std::string texPath = "diablo3_pose_diffuse.bmp";
    Model model(objPath, texPath);
    //设置光源
    Light light1({ 30, 30, 30, 1.f }, { 1000, 1000, 1000 });
    Light light2({ -20, 20, 0, 1.f }, { 500, 500, 500 });
    std::vector<Light> lights = {light1, light2};
    //设置摄像机位置，模型变换参数和投影参数
    Vector3f scalar(3.f, 3.f, 3.f);
    Vector3f axis(0.0f, 1.0f, 0.0f);
    Vector3f translation(0.0f, 0.0f, 0.0f);
    float angel = 0.0f;
    Vector3f eyePos(0.0f, 0.0f, 10.f);
    Vector3f eyeLookat(0.0f, 0.0f, -1.0f);
    Vector3f upOrient(0.0f, 1.0f, 0.0f);
    float eye_fov = 45.0f, aspect_ratio = SRC_WIDTH / SRC_HEIGHT, zNear = -0.1f, zFar = -50.0f;
    Transformation trans;
    model.setModelMatrix(Transformation::setTranslation(translation), Transformation::setRotation(angel, axis), Transformation::setScalar(scalar));
    trans.setViewMatrix(eyePos, eyeLookat, upOrient);
    trans.setProjectionMatrix(eye_fov, aspect_ratio, zNear, zFar);
    //开始渲染
    Rasterizer rs(SRC_WIDTH, SRC_HEIGHT, {0.f, 0.f, 0.f});
    rs.rendering(model, lights, trans);
    //渲染结果打印成图片
    cv::Mat image(SRC_WIDTH, SRC_HEIGHT, CV_32FC3, rs.frame_buffer.data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imwrite("output.png", image);

    return 0;
}
