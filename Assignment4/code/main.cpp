#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        //emplace_back() 和 push_back() 的区别，就在于底层实现的机制不同。
        //push_back() 向容器尾部添加元素时，首先会创建这个元素，然后再将这个
        //元素拷贝或者移动到容器中（如果是拷贝的话，事后会自行销毁先前创建的
        //这个元素）；而 emplace_back() 在实现时，则是直接在容器尾部创建这个
        //元素，省去了拷贝或移动元素的过程。
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
    std::vector<cv::Point2f> points = control_points;
    //当序列控制点大于等于3个点需要迭代
    while(points.size() >= 3)
    {
        //每次迭代控制点个数会减1
        std::vector<cv::Point2f> points_temp;
        //根据当前控制点，构成线段，并根据t值分配t和（1-t）
        for(int i = 0; i < points.size() - 1; i++)
        {
            cv::Point2f first = points[i];
            cv::Point2f second = points[i + 1];
            cv::Point2d mid = (1 - t) * first + t * second;
            points_temp.push_back(mid);
        }
        points = points_temp;
    }
    //控制点个数只有两个
    cv::Point2f start = points[0];
    cv::Point2f end = points[points.size() - 1];
    cv::Point2d mid = (1 - t) * start + t * end;

    return mid;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; 

        //反走样
        float x = point.x - std::floor(point.x);
        float y = point.y - std::floor(point.y);
        int x_flag = x < 0.5f ? -1 : 1;
        int y_flag = y < 0.5f ? -1 : 1;

        //距离采样点最近的4个像素（像素中心）
        cv::Point2f p00 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p01 = cv::Point2f(std::floor(point.x + x_flag * 1.0f) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p10 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y + y_flag * 1.0f) + 0.5f);
        cv::Point2f p11 = cv::Point2f(std::floor(point.x + x_flag * 1.0f) + 0.5f, std::floor(point.y + y_flag * 1.0f) + 0.5f);

        std::vector<cv::Point2f> vec;
        vec.push_back(p01);
        vec.push_back(p10);
        vec.push_back(p11);

        //计算采样点与最近像素中心的距离
        cv::Point2f distance = p00 - point;
        float len = sqrt(distance.x * distance.x + distance.y * distance.y);

        //对其它三个像素点进行着色
        for(auto p : vec)
        {
            //根据距离比，计算着色系数
            cv::Point2f d = p - point;
            float l = sqrt(d.x * d.x + d.y * d.y);
            float percent = len / l;
            cv::Vec3d color = window.at<cv::Vec3b>(p.y, p.x);
            //根据系数调整绿色的比例
            color[1] = std::max(color[1], (double)255 * percent);
            window.at<cv::Vec3b>(p.y, p.x) = color;
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
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

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
