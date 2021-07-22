#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#define  ANGLE_TOR_ADIAN(a) a/(45.0 / atan(1.0))    //弧度_到_角度 的 比率
constexpr double MY_PI = 3.1415926;

//视觉变换矩阵        view矩阵就是以摄像机位置为原点，朝向为坐标轴建立新的坐标系 
//这个view就是和世界坐标轴对齐的，没有旋转，只有平移
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 
        1, 0, 0, -eye_pos[0],       //平移操作
        0, 1, 0, -eye_pos[1], 
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

//模型变换矩阵
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f translateZ;
    //  cos(rotation_angle), -sin(rotation_angle), 0, 0,
    //	sin(rotation_angle), cos(rotation_angle), 0, 0,
    //	0, 0, 1, 0,
    //	0, 0, 0, 1;
    float radian = ANGLE_TOR_ADIAN(rotation_angle);

    model(0, 0) = std::cos(radian);
    model(0, 1) = -std::sin(radian);
    model(1, 0) = std::sin(radian);
    model(1, 1) = std::cos(radian);

    return model;
}

//透视投影矩阵 透视矩阵=正交矩阵*透视矩阵->正交矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    //正交矩阵 单位矩阵
    Eigen::Matrix4f orth = Eigen::Matrix4f::Identity();
    //透视矩阵->正交矩阵
    Eigen::Matrix4f pertoorth = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    
    //正交投影  投影变换是根据定义，由若干个不同的矩阵变换相乘得到的，最后投影矩阵是各个变换矩阵相乘的结果
    /*
    eye_fov:纵向的视角大小
    aspect_ratio:裁剪面的宽高比
    zNear : 近裁剪面离摄像机的距离
    zFar : 远裁剪面离摄像机的距离
    */

  
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
    float u =axis.x();
    float v = axis.y();
    float w = axis.z();

    rotation <<
        pow(u,2)+(1-pow(u,2))*cos(angle),u*v*(1-cos(angle))-w*sin(angle), u* w* (1 - cos(angle)) + v * sin(angle),0,
        u* v* (1 - cos(angle)) + w * sin(angle), pow(v, 2) + (1 - pow(v, 2)) * cos(angle), v* w* (1 - cos(angle)) - u * sin(angle),0,
        u* w* (1 - cos(angle)) - v * sin(angle),v*w*(1-cos(angle))+u*sin(angle), pow(w, 2) + (1 - pow(w, 2)) * cos(angle),0,
        0, 0, 0, 1;

    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
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

    Vector3f axis{ 10,20,15 };
    float rotationAngle{ 45 };
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_rotation(get_rotation(axis, rotationAngle));

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
        r.set_rotation(get_rotation(axis, rotationAngle));

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

    return 0;
}
