#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 6) 
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
    if (control_points.size() == 1)
        return control_points[0];  //the last point to draw the curves

    std::vector<cv::Point2f> points;
    for (int i = 0; i < control_points.size()-1; i++) {
        points.push_back((1 - t) * control_points[i] + t * control_points[i + 1]);
    }

    return recursive_bezier(points,t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0; t <= 1; t += 0.0001) {
        cv::Point2f point = recursive_bezier(control_points, t);//calculate the point on curves
        window.at<cv::Vec3b>(point.y,point.x)[1] = 255;//set color in green (0,255,0)

        //Anti-aliasing
        float x = point.x - std::floor(point.x);
        float y = point.y - std::floor(point.y);
        int x_flag = x < 0.5f ? -1 : 1;
        int y_flag = y < 0.5f ? -1 : 1;

        //sampling 4 nearest pixel
        cv::Point2f p00 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p01 = cv::Point2f(std::floor(point.x + x_flag * 1.0f) + 0.5f, std::floor(point.y) + 0.5f);
        cv::Point2f p10 = cv::Point2f(std::floor(point.x) + 0.5f, std::floor(point.y + y_flag * 1.0f) + 0.5f);
        cv::Point2f p11 = cv::Point2f(std::floor(point.x + x_flag * 1.0f) + 0.5f, std::floor(point.y + y_flag * 1.0f) + 0.5f);

        std::vector<cv::Point2f> vec{p01,p10,p11};

        // calculate the nearest distance 
        cv::Point2f dis  = p00 - point;
        float len = sqrt(dis.x * dis.x + dis.y * dis.y);

        //shade the pixel with the distance
        for (auto p : vec)
        {
            cv::Point2f d = p - point;
            float l = sqrt(d.x * d.x + d.y * d.y);//calculate the distance
            float percent = len / l;
            //set the point color
            cv::Vec3d color = window.at<cv::Vec3b>(p.y, p.x);
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

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);//set the control point

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {0, 255, 0}, 3);//draw control points
        }

        if (control_points.size() == 6) // set six control points
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);//draw bezier curves

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve_Ati.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
