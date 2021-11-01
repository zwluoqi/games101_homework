#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle)
{
            float n1 = axis[0];
            float n2 = axis[1];
            float n3 = axis[2];
            float cosAngle = cos(angle);
            float sinAngle = sin(angle);
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    rotate <<   n1*n1 * (1 - cosAngle) + cosAngle, n1*n2*(1 - cosAngle) - n3*sinAngle, n1*n3*(1 - cosAngle) + n2*sinAngle, 0,
                n1*n2*(1 - cosAngle) + n3*sinAngle, n2*n2 * (1 - cosAngle) + cosAngle, n2*n3*(1 - cosAngle) - n1*sinAngle, 0,
                n1*n3*(1 - cosAngle) - n2*sinAngle, n2*n3*(1 - cosAngle) + n1*sinAngle, n3*n3 * (1 - cosAngle) + cosAngle, 0,
                0, 0, 0, 1;//单纯实现了关于z轴的旋转矩阵

    return rotate;

}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate;
    float radian = rotation_angle/180.0*MY_PI;
    rotate << cos(radian), -1*sin(radian), 0, 0,
              sin(radian), cos(radian), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;//单纯实现了关于z轴的旋转矩阵
    model = rotate * model; 
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    //Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();//将透视投影转换为正交投影的矩阵
    P2O<<zNear, 0, 0, 0,
         0, zNear, 0, 0,
         0, 0, zNear+zFar,(-1)*zFar*zNear,
         0, 0, 1, 0;// 进行透视投影转化为正交投影的矩阵
    float halfEyeAngelRadian = eye_fov/2.0/180.0*MY_PI;
    float t = zNear*std::tan(halfEyeAngelRadian);//top y轴的最高点
    float r=t*aspect_ratio;//right x轴的最大值
    float l=(-1)*r;//left x轴最小值
    float b=(-1)*t;//bottom y轴的最大值
    Eigen::Matrix4f ortho1=Eigen::Matrix4f::Identity();
    ortho1<<2/(r-l),0,0,0,
        0,2/(t-b),0,0,
        0,0,2/(zNear-zFar),0,
        0,0,0,1;//进行一定的缩放使之成为一个标准的长度为2的正方体
    Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Identity();
    ortho2<<1,0,0,(-1)*(r+l)/2,
        0,1,0,(-1)*(t+b)/2,
        0,0,1,(-1)*(zNear+zFar)/2,
        0,0,0,1;// 把一个长方体的中心移动到原点
    Eigen::Matrix4f Matrix_ortho = ortho1 * ortho2;
    projection = Matrix_ortho * P2O;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        std::cout << "argc " <<argc<< '\n';
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else{
            std::cout << "default png " << '\n';
            //return 0;
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        std::cout << "command_line " << '\n';
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }
    std::cout << "over " << '\n';
    return 0;
}
