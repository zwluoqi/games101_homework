//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
// #include <Eigen/Eigen>

#include "Texture.hpp"


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        diff_texture = nullptr;
        normal_texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex, Texture* nor_tex) :
         color(col), normal(nor), tex_coords(tc), diff_texture(tex), normal_texture(nor_tex){}


    Eigen::Vector3f view_pos;
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;
    Texture* diff_texture;
    Texture* normal_texture;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
