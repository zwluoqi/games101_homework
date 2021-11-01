//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
// #include <Eigen/Eigen>

#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = clamp(u * width,width-1);
        auto v_img = clamp((1 - v) * height,height-1);
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColor0(int u_img, int v_img) 
    {
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    int clamp(int v,int max) {
        return std::clamp(v, 0, max-1);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        auto u1_image = clamp( (int)u_img - 1,width);
        auto u2_image = clamp((int)u_img + 1,height);

        auto v1_image = clamp((int)v_img - 1,width);
        auto v2_image = clamp((int)v_img + 1,height);


        auto u1v1 = getColor0(u1_image, v1_image);
        auto u2v1 = getColor0(u2_image, v1_image);

        auto u1v2 = getColor0(u1_image, v2_image);
        auto u2v2 = getColor0(u2_image, v2_image);

        Eigen::Vector3f f1 = (u2_image - u_img) / (u2_image - u1_image) * u1v1 + (u_img - u1_image) / (u2_image - u1_image) * u2v1;
        Eigen::Vector3f f2 = (u2_image - u_img) / (u2_image - u1_image) * u1v2 + (u_img - u1_image) / (u2_image - u1_image) * u2v2;

        Eigen::Vector3f f = (v2_image - v_img) / (v2_image - v1_image) * f1 + (v_img - v1_image) / (v2_image - v1_image) * f2;
        return f;
    }
};
#endif //RASTERIZER_TEXTURE_H
