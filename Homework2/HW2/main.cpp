// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

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

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotate;
    float radian = rotation_angle / 180.0 * MY_PI;
    rotate << cos(radian), -1 * sin(radian), 0, 0,
        sin(radian), cos(radian), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;//����ʵ���˹���z�����ת����
    model = rotate * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();//��͸��ͶӰת��Ϊ����ͶӰ�ľ���
    P2O << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, (-1)* zFar* zNear,
        0, 0, 1, 0;// ����͸��ͶӰת��Ϊ����ͶӰ�ľ���
    float halfEyeAngelRadian = eye_fov / 2.0 / 180.0 * MY_PI;
    float t = zNear * std::tan(halfEyeAngelRadian);//top y�����ߵ�
    float r = t * aspect_ratio;//right x������ֵ
    float l = (-1) * r;//left x����Сֵ
    float b = (-1) * t;//bottom y������ֵ
    Eigen::Matrix4f ortho1 = Eigen::Matrix4f::Identity();
    ortho1 << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;//����һ��������ʹ֮��Ϊһ����׼�ĳ���Ϊ2��������
    Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Identity();
    ortho2 << 1, 0, 0, (-1)* (r + l) / 2,
        0, 1, 0, (-1)* (t + b) / 2,
        0, 0, 1, (-1)* (zNear + zFar) / 2,
        0, 0, 0, 1;// ��һ��������������ƶ���ԭ��
    Eigen::Matrix4f Matrix_ortho = ortho1 * ortho2;
    projection = Matrix_ortho * P2O;

    return projection;
}

const int width = 700;
const int height = 700;

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    int sample_frequency = 0;
    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
        if(argc >=3){
            sample_frequency = std::stoi(argv[2]);
        }
    }

    rst::rasterizer r(width, height);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 12.0, 185.0},
                    {217.0, 238.0, 12.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 12.0, 238.0},
                    {185.0, 217.0, 12.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.setSSAANum(sample_frequency);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(width, height, CV_32FC3, r.frame_buffer().data());
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
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(width, height, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on