#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;


void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 16) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> new_control_points;
    for (int i = 0; i < control_points.size()-1; i++) {
        cv::Point2f new_point = control_points[i + 1] * t + control_points[i] * (1 - t);
        new_control_points.push_back(new_point);
    }
    if (new_control_points.size() == 1) {
        return new_control_points[0];
    }
    else {
        return recursive_bezier(new_control_points, t);
    }

    return cv::Point2f();

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    int row = window.rows;
    int col = window.cols;

    float p_row = 1.0 / row;
    float startx = 0;
    float endx = row;
    for (int i = 0; i < control_points.size(); i++) {
        auto cp = control_points[i];
        if (cp.x>startx) {
            startx = cp.x;
        }
        if (cp.x < endx) {
            endx = cp.x;
        }
    }
    float length = endx - startx;
    
    for (float t = 0; t< 1; t+= p_row) {
        auto p = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(p.y, p.x)[1] = 255;
    }
}

int main() 
{

    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::Mat source = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));

    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() >= 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            //key = cv::waitKey(0);

            //return 0;
        }
        else {

            cv::imshow("Bezier Curve", window);
        }
        source.copyTo(window);
        key = cv::waitKey(20);
    }

return 0;
}
