#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#if _WIN32 || WIN32  //windows??
#include<Windows.h> 
#endif

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
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

//贝塞尔用递归最好实现
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    auto count = control_points.size();
    std::vector<cv::Point2f > pointsBezier;
    if (count == 1) {
        return control_points[0];
    }

    for (size_t i = 0; i < count - 1; i++)
    {
        auto Pt = (1 - t) * control_points[i] + t * control_points[i + 1];
        pointsBezier.push_back(Pt);
    }

    return recursive_bezier(pointsBezier, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto p = recursive_bezier(control_points, t);


        float dx = p.x - std::floor(p.x);
        float dy = p.y - std::floor(p.y);
        cv::Point2f near_pixel;
        cv::Point2f p00;
        cv::Point2f p01;
        cv::Point2f p10;
        cv::Point2f p11;

        if (dx <= 0.5f && dy <= 0.5f) {
            p00 = cv::Point2f(std::floor(p.x), std::floor(p.y));
            p01 = cv::Point2f(std::floor(p.x), std::floor(p.y) + 1);
            p10 = cv::Point2f(std::floor(p.x) + 1, std::floor(p.y));
            p11 = cv::Point2f(std::floor(p.x) + 1, std::floor(p.y) + 1);
            near_pixel = p00;
        }
        if (dx <= 0.5f && dy > 0.5f) {
            p00 = cv::Point2f(std::floor(p.x) - 1, std::floor(p.y));
            p01 = cv::Point2f(std::floor(p.x) - 1, std::floor(p.y) + 1);
            p10 = cv::Point2f(std::floor(p.x), std::floor(p.y));
            p11 = cv::Point2f(std::floor(p.x), std::floor(p.y) + 1);
            near_pixel = p10;
        }
        if (dx > 0.5f && dy <= 0.5f) {
            p00 = cv::Point2f(std::floor(p.x), std::floor(p.y) - 1);
            p01 = cv::Point2f(std::floor(p.x), std::floor(p.y));
            p10 = cv::Point2f(std::floor(p.x) + 1, std::floor(p.y) - 1);
            p11 = cv::Point2f(std::floor(p.x) + 1, std::floor(p.y));
            near_pixel = p01;
        }
        if (dx > 0.5f && dy > 0.5f) {
            p00 = cv::Point2f(std::floor(p.x) - 1, std::floor(p.y) - 1);
            p01 = cv::Point2f(std::floor(p.x) - 1, std::floor(p.y));
            p10 = cv::Point2f(std::floor(p.x), std::floor(p.y) - 1);
            p11 = cv::Point2f(std::floor(p.x), std::floor(p.y));
            near_pixel = p11;
        }

        std::vector<cv::Point2f> vec;
        vec.push_back(p00);
        vec.push_back(p01);
        vec.push_back(p10);
        vec.push_back(p11);

        float distance = sqrt((p - near_pixel).x * (p - near_pixel).x + (p - near_pixel).y * (p - near_pixel).y);

        for (auto point : vec) {
            float d = sqrt((p - point).x * (p - point).x + (p - point).y * (p - point).y);
            float ratio = distance / d;
            window.at<cv::Vec3b>(p.y, p.x)[1] = 255;
            cv::Vec3b color = window.at<cv::Vec3b>(p.y, p.x);
            window.at<cv::Vec3b>(point.y, point.x) = ratio * color;
        }
    }
  
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {174, 174, 0}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            //   bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
