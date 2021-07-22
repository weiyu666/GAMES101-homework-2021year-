#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#define  ANGLE_TOR_ADIAN(a) a/(45.0 / atan(1.0))    //����_��_�Ƕ� �� ����
constexpr double MY_PI = 3.1415926;

//�Ӿ��任����        view��������������λ��Ϊԭ�㣬����Ϊ�����Ὠ���µ�����ϵ 
//���view���Ǻ��������������ģ�û����ת��ֻ��ƽ��
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 
        1, 0, 0, -eye_pos[0],       //ƽ�Ʋ���
        0, 1, 0, -eye_pos[1], 
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

//ģ�ͱ任����
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

//͸��ͶӰ���� ͸�Ӿ���=��������*͸�Ӿ���->��������
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    //�������� ��λ����
    Eigen::Matrix4f orth = Eigen::Matrix4f::Identity();
    //͸�Ӿ���->��������
    Eigen::Matrix4f pertoorth = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    
    //����ͶӰ  ͶӰ�任�Ǹ��ݶ��壬�����ɸ���ͬ�ľ���任��˵õ��ģ����ͶӰ�����Ǹ����任������˵Ľ��
    /*
    eye_fov:������ӽǴ�С
    aspect_ratio:�ü���Ŀ�߱�
    zNear : ���ü�����������ľ���
    zFar : Զ�ü�����������ľ���
    */

  
    float halfEyeAngelRadian = eye_fov / 2 / 180.0 * MY_PI;
    //�����ֻ��znear��zfar���������꣬��Ϊ��z�Ḻ���������Ǹ���
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


//����������ת  ��ת
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
