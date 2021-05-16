// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(double x, double y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f p;
    p << x, y;

    //假设_v[0] = A, _v[1] = B, _v[2] = C
    Eigen::Vector2f AB = _v[1].head(2) - _v[0].head(2);     //向量AB，前两个元素，即（x,y）坐标
    Eigen::Vector2f BC = _v[2].head(2) - _v[1].head(2);     //向量BC
    Eigen::Vector2f CA = _v[0].head(2) - _v[2].head(2);     //向量CA

    Eigen::Vector2f AP = p - _v[0].head(2);     //向量AP
    Eigen::Vector2f BP = p - _v[1].head(2);     //向量BP
    Eigen::Vector2f CP = p - _v[2].head(2);     //向量CP

    return AB[0] * AP[1] - AB[1] * AP[0] > 0 
        && BC[0] * BP[1] - BC[1] * BP[0] > 0
        && CA[0] * CP[1] - CA[1] * CP[0] > 0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

//draw的作用是给一个大小为 width * height 的 frame_buf 的每个像素设定颜色（RGB）
//当有多个三角形并有重叠时，会进行 Depth Test, 此外这里进行了模糊处理操作
//提供给CV函数进行绘图
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        //my test
        // for(auto& myVec : v)
        // {
        //     std::cout<<"After MVP"<<std::endl;
        //     std::cout<<myVec.z()<<std::endl;
        // }

        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }

        //my test
        // for(auto& myVec : v)
        // {
        //     std::cout<<"After division"<<std::endl;
        //     std::cout<<myVec.z()<<std::endl;
        // }

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        //my test
        // for(auto& myVec : v)
        // {
        //     std::cout<<"After viewport"<<std::endl;
        //     std::cout<<myVec.z()<<std::endl;
        // }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        //Depth Test 
        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    //1. 创建三角形的 2 维 bounding box。
    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));
    
    //floor：向下取整， cel:向上取整
    min_x = (int)std::floor(min_x);
    max_x = (int)std::ceil(max_x);
    min_y = (int)std::floor(min_y);
    max_y = (int)std::ceil(max_y);

    //Super-Sampling Anti-Aliasing的英文缩写，指超采样抗锯齿
    bool SSAA = true;      //是否开启SSAA，也就是是否用super-sampling处理Anti-aliasing, 

    if(SSAA)
    {
        //这里使用SSAA 2X2
        std::vector<Eigen::Vector2f> pos
        {
            {0.25, 0.25},
            {0.75, 0.25},
            {0.25, 0.75},
            {0.75, 0.75}
        };

        //遍历此 bounding box 内的所有像素（使用其整数索引）。然后，使用像素中心的屏幕空间坐标来检查中心点是否在三角形内。
        for(int x = min_x; x <= max_x; x++){
            for(int y = min_y; y <= max_y; y++){             
                float minDepth =  FLT_MAX;  //记录当前像素里的子像素点（这些子像素点需要满足在三角形内）的最小深度值（越靠近摄像机）
                int current_index = get_index(x, y) * 4;     
                for(int i = 0; i < 4; i++){
                    if(insideTriangle((float)x + pos[i][0], (float)y + pos[i][1], t.v)){
                        //如果在，对深度z进行插值
                        auto tup = computeBarycentric2D((float)x + pos[i][0], (float)y + pos[i][1], t.v);
                        float alpha, beta, gamma;
                        //std::tie会将变量的引用整合成一个tuple，从而实现批量赋值。
                        std::tie(alpha, beta, gamma) = tup;
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        minDepth = std::min(minDepth, z_interpolated);

                        if(depth_buf[current_index + i] > z_interpolated){
                            depth_buf[current_index + i] = z_interpolated;
                            color_buf[current_index + i] = t.getColor();
                        }
                    }
                }
                Vector3f color = Vector3f(0, 0, 0);
                //累加当前像素的所有子像素的颜色平均值
                for(int i = 0; i < 4; i++){
                    color += color_buf[current_index + i] / 4.0f; 
                }               
                Vector3f point(3);
                //这里的minDepth没有用处，1是set_pixel()不需要用到深度值，2是现在frame_buf的值取决于color_buf
                point << (float)x, (float)y, minDepth;       
                set_pixel(point, color);
            }
        }
        
    }
    //没有抗锯齿操作
    else{
        for(int x = min_x; x <= max_x; x++){
            for(int y = min_y; y <= max_y; y++){
                if(insideTriangle((float)x + 0.5, (float)y + 0.5, t.v)){
                    auto tup = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;      

                    if(depth_buf[get_index(x, y)] > z_interpolated){
                        //如果当前点更靠近相机，请设置像素颜色并更新深度缓冲区 (depth buffer)。
                        Vector3f color = t.getColor();
                        Vector3f point(3);
                        point << (float)x, (float)y, z_interpolated;
                        //修改颜色
                        set_pixel(point, color);
                        //更新深度
                        depth_buf[get_index(x, y)] = z_interpolated;   
                    }                
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(color_buf.begin(), color_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    w = w * 4 + 100;    //100是为了防爆
    depth_buf.resize(w * h);
    color_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on