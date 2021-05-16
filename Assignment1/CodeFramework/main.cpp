#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

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

    Eigen::Matrix4f rotation;
    float radian = rotation_angle / 180.0 * MY_PI;
    rotation << cos(radian), -sin(radian), 0, 0,
        sin(radian), cos(radian), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f persp2ortho, ortho_translate, ortho_scale, ortho;

    float radian = eye_fov / 2 / 180 * MY_PI;
    float top = tan(radian) * zNear;        //top
    float right = aspect_ratio * top;       //right
    float bottom = -top;                    //bottom
    float left = -right;                    //left

    //make a matrix that represents persp->ortho
    persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -1.0 * zNear * zFar,
        0, 0, 1, 0;

    //make a matrix that represnets ortho
    //translate
    ortho_translate << 1, 0, 0, -(right + left) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;
    //scale
    ortho_scale << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    ortho = ortho_scale * ortho_translate;

    //make a projection matrix
    projection = ortho * persp2ortho * projection;

    return projection;
}

// Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
// {
//     Eigen::Matrix4f  projection;
//     float angle = eye_fov / 360 * MY_PI;
//     projection << 1 / (aspect_ratio * tan(angle)), 0, 0, 0, 0, 1/tan(angle), 0, 0, 0, 0,
//     (zNear+zFar) / (zFar-zNear), 2*zNear*zFar / (zNear-zFar), 0, 0, 1, 0;
//     Eigen::Matrix4f p;
//     p<<-1,0,0,0,
//     0,-1,0,0,
//     0,0,-1,0,
//     0,0,0,1;
//     return p* projection;
// }

//该函数的作用是得到绕任意过原点的轴的旋转变换矩阵
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float radian = angle / 180.0 * MY_PI;
    float norm = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[0] /= norm;
    axis[1] /= norm;
    axis[2] /= norm;

    Eigen::Matrix3f N(3, 3);
    N << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;

    Eigen::Matrix3f component1 = Eigen::Matrix3f::Identity() * cos(radian);
    Eigen::Matrix3f component2 = axis * axis.transpose() * (1 - cos(radian));
    Eigen::Matrix3f component3 = N * sin(radian);

    Eigen::Matrix3f rotate3f = component1 + component2 + component3;

    Eigen::Matrix4f rotate4f= Eigen::Matrix4f::Identity();
    rotate4f.block(0, 0, 3, 3) = rotate3f;

    model = rotate4f * model;

    return model;
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
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    //std::vector<Eigen::Vector3f> pos{{2, 0, 0}, {0, 2, 0}, {-2, 0, 0}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
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

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation({0, 0, 1}, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 45;
        }
        else if (key == 'd') {
            angle -= 45;
        }
    }

    return 0;
}
