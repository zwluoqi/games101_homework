#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();
    P2O << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, (-1)* zFar* zNear,
        0, 0, 1, 0;
    float halfEyeAngelRadian = eye_fov / 2.0 / 180.0 * MY_PI;
    float t = zNear * std::tan(halfEyeAngelRadian);
    float r = t * aspect_ratio;
    float l = (-1) * r;
    float b = (-1) * t;
    Eigen::Matrix4f ortho1 = Eigen::Matrix4f::Identity();
    ortho1 << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;
    Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Identity();
    ortho2 << 1, 0, 0, (-1)* (r + l) / 2,
        0, 1, 0, (-1)* (t + b) / 2,
        0, 0, 1, (-1)* (zNear + zFar) / 2,
        0, 0, 0, 1;
    Eigen::Matrix4f Matrix_ortho = ortho1 * ortho2;
    projection = Matrix_ortho * P2O;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.diff_texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment

        return_color = payload.diff_texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);

    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal.normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f lightDis = light.position - point;

        auto lightDir = lightDis.normalized();
        Eigen::Vector3f viewDir = -point;
        viewDir = viewDir.normalized();
        Eigen::Vector3f halfDir = (lightDir + viewDir);
        halfDir = halfDir.normalized();
        // Eigen::Vector3f ambient = ka;

        auto nl = normal.dot(lightDir);
        auto nh = normal.dot(halfDir);
        Eigen::Vector3f intensity = light.intensity / lightDis.dot(lightDis);
        Eigen::Vector3f diffColor = kd.cwiseProduct(intensity);
        Eigen::Vector3f speColor = ks.cwiseProduct(intensity);
        Eigen::Vector3f diffuse = std::fmax(0.0, nl) * diffColor;
        Eigen::Vector3f specular = std::pow(std::fmax(0.0, nh), p) * speColor;
        Eigen::Vector3f aintensity = amb_light_intensity;
        Eigen::Vector3f amb = ka.cwiseProduct(aintensity);
        result_color += diffuse + specular + amb;
    }

    return result_color * 255.f;
}

// static float max(float a,float b){
//     return a>b?a:b;
// }

//static auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
//{
//    return Vector4f(v3.x(), v3.y(), v3.z(), w);
//}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    //auto model = get_model_matrix(p);
    //auto view = get_view_matrix(eye_pos);


    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal.normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        //auto lighViewPos = view * model * to_vec4(light.position, 1.0);
        Eigen::Vector3f lightDis = light.position - point;

        auto lightDir = lightDis.normalized();
        Eigen::Vector3f viewDir = -point;
        viewDir = viewDir.normalized();
        Eigen::Vector3f halfDir = (lightDir+viewDir);
        halfDir = halfDir.normalized();
        // Eigen::Vector3f ambient = ka;

        auto nl = normal.dot(lightDir);
        auto nh = normal.dot(halfDir);
        Eigen::Vector3f intensity = light.intensity / lightDis.dot(lightDis);
        Eigen::Vector3f diffColor = kd.cwiseProduct(intensity);
        Eigen::Vector3f speColor = ks.cwiseProduct(intensity);
        Eigen::Vector3f diffuse = std::fmax(0.0, nl)* diffColor;
        Eigen::Vector3f specular = std::pow(std::fmax(0.0, nh), p) * speColor;
        Eigen::Vector3f aintensity = amb_light_intensity;
        Eigen::Vector3f amb = ka.cwiseProduct(aintensity);
        result_color += diffuse + specular + amb;
    }

    return result_color * 255;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    float x = normal[0];
    float y = normal[1];
    float z = normal[2];
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];

     Eigen::Vector3f n = normal;
     Eigen::Vector3f t = Eigen::Vector3f(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
     Eigen::Vector3f b = n.cross(t);// n cross product t
     Eigen::Matrix3f TBN = Eigen::Matrix3f::Identity();
     TBN << t
         , b
         , n;

     auto u_peroid = 1.0 / payload.normal_texture->width;
     auto v_peroid = 1.0 / payload.normal_texture->height;
     float dU = kh * kn * (payload.normal_texture->getColor(u + u_peroid, v).norm() - payload.normal_texture->getColor(u, v).norm());
     float dV = kh * kn * (payload.normal_texture->getColor(u, v + v_peroid).norm() - payload.normal_texture->getColor(u, v).norm());
     Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1).normalized();
     point = point + kn * n * payload.normal_texture->getColor(u,v).norm();
     normal = (TBN * ln).normalized();


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f lightDis = light.position - point;

        auto lightDir = lightDis.normalized();
        Eigen::Vector3f viewDir = -point;
        viewDir = viewDir.normalized();
        Eigen::Vector3f halfDir = (lightDir+viewDir);
        halfDir = halfDir.normalized();
        // Eigen::Vector3f ambient = ka;

        auto nl = normal.dot(lightDir);
        auto nh = normal.dot(halfDir);
        Eigen::Vector3f intensity = light.intensity / lightDis.dot(lightDis);
        Eigen::Vector3f diffColor = kd.cwiseProduct(intensity);
        Eigen::Vector3f speColor = ks.cwiseProduct(intensity);
        Eigen::Vector3f diffuse = std::fmax(0.0, nl)* diffColor;
        Eigen::Vector3f specular = std::pow(std::fmax(0.0, nh), p) * speColor;
        Eigen::Vector3f aintensity = amb_light_intensity;
        Eigen::Vector3f amb = ka.cwiseProduct(aintensity);
        result_color += diffuse + specular + amb;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    Eigen::Vector3f p_normal = (normal + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2;

    float kh = 0.2, kn = 0.1;

    Eigen::Vector3f normal_color = { 0, 0, 0 };
    if (payload.normal_texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment

        normal_color = payload.normal_texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);

    }

    // TODO: Implement bump mapping here
    float x = normal[0];
    float y = normal[1];
    float z = normal[2];
    float u = payload.tex_coords[0];
    float v = payload.tex_coords[1];

     Eigen::Vector3f n = normal;
     Eigen::Vector3f t = Eigen::Vector3f(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
     Eigen::Vector3f b = n.cross(t);// n cross product t
     Eigen::Matrix3f TBN = Eigen::Matrix3f::Identity();
     TBN << t
         , b
         , n;
     //TBN = TBN.transpose();
     //Eigen::Matrix TBN = [t b n]
     ;
     auto u_peroid = 1.0 / payload.normal_texture->width;
     auto v_peroid = 1.0 / payload.normal_texture->height;
     float dU = kh * kn * (payload.normal_texture->getColor(u + u_peroid, v).norm() - payload.normal_texture->getColor(u, v).norm());
     float dV = kh * kn * (payload.normal_texture->getColor(u, v + v_peroid).norm() - payload.normal_texture->getColor(u, v).norm());
     Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1).normalized();
     auto c_normal = (TBN * ln).normalized();
    //  c_normal = (c_normal + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2;

     ////normal mapping
     //normal_color = 2 * normal_color/255.0 - Vector3f(1.0,1.0,1.0);
     //normal = (TBN * normal_color).normalized();
     //auto c_normal = (normal + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2;

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = c_normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = -30.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/Marry/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/Marry/Marry.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {        
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }

            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_normaltexture(Texture(obj_path + texture_path));
    auto diffuse = Texture(obj_path + "Marry_diff.png");
    r.set_diffusetexture(diffuse);

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    command_line = true;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
           
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,1.5,6};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        std::cout << "command_line\n";
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
