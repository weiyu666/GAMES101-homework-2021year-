// clang-format off
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

#define  ANGLE_TOR_ADIAN(a) a/(45.0 / atan(1.0))    //弧度_到_角度 的 比率
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
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //正交矩阵 单位矩阵
    Eigen::Matrix4f orth = Eigen::Matrix4f::Identity();
    //透视矩阵->正交矩阵
    Eigen::Matrix4f pertoorth = Eigen::Matrix4f::Identity();

    float halfEyeAngelRadian = eye_fov / 2 / 180.0 * MY_PI;
    //传入的只是znear和zfar，不是坐标，因为是z轴负半轴所以是负的
    float n = -1 * zNear;
    float f = -1 * zFar;

    float t = zNear * std::tan(halfEyeAngelRadian);// top / znear = tan(halfEyeAngelRadian)
    float r = t * aspect_ratio;// top / right = aspect_ration
    float l = (-1) * r;//
    float b = (-1) * t;//
    orth << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / zNear - zFar, 0,
        0, 0, 0, 1;
    pertoorth << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -1 * n * f,
        0, 0, 1, 0;
    projection = orth * pertoorth;


    return projection;
}


//沿任意轴旋转  旋转
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    float u = axis.x();
    float v = axis.y();
    float w = axis.z();

    rotation <<
        pow(u, 2) + (1 - pow(u, 2)) * cos(angle), u* v* (1 - cos(angle)) - w * sin(angle), u* w* (1 - cos(angle)) + v * sin(angle), 0,
        u* v* (1 - cos(angle)) + w * sin(angle), pow(v, 2) + (1 - pow(v, 2)) * cos(angle), v* w* (1 - cos(angle)) - u * sin(angle), 0,
        u* w* (1 - cos(angle)) - v * sin(angle), v* w* (1 - cos(angle)) + u * sin(angle), pow(w, 2) + (1 - pow(w, 2)) * cos(angle), 0,
        0, 0, 0, 1;

    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

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
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    Vector3f axis{ 10,20,15 };
    float rotationAngle{ 45 };

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
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
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_rotation(get_rotation(axis, rotationAngle));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on