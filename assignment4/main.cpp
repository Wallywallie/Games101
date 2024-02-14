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
        //cv::Vec3b is a vector composed of 3 'uchar', namely the B G R channel.(rgb)
        //it sets the trird channel(RED channel) at the specific point to 255,namely sets its color to Red.
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2) {
        return cv::Point2f((1 - t) * control_points[0] + t * control_points[1]);
    }
    std::vector<cv::Point2f> newPonts;
    for (int i = 1; i < control_points.size(); i++) {
        newPonts.push_back((1 - t) *control_points[i - 1] + t * control_points[i]);
    }  
    return recursive_bezier(newPonts, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    float Interval = 0.001;
    for (float t = 0.0; t <= 1.0; t += Interval) {
        auto sample_point = recursive_bezier(control_points, t);

        //find the four nearest pixel
        int xmax = (sample_point.x - std::floor(sample_point.x) < 0.5) ? std::floor(sample_point.x) : std::ceil(sample_point.x);
        int ymax = (sample_point.y - std::floor(sample_point.y) < 0.5) ? std::floor(sample_point.y) : std::ceil(sample_point.y);
        cv::Point2f p1(xmax - 0.5, ymax - 0.5);
        cv::Point2f p2(xmax - 0.5, ymax + 0.5);
        cv::Point2f p3(xmax + 0.5, ymax - 0.5);
        cv::Point2f p4(xmax + 0.5, ymax + 0.5);        
        std::vector<cv::Point2f> points = {p1, p2, p3, p4};
        double sum_distance = 0;
        double max_d = std::sqrt(2);
        std::vector<double> dis;
        for (auto p : points) {
            double distance = max_d - std::sqrt( (p.x - sample_point.x) * (p.x - sample_point.x) + (p.y - sample_point.y) * (p.y - sample_point.y));           
            dis.push_back(distance);
            sum_distance += distance;

        }
        for (int i = 0; i < 4; i++) {
            double color = std::min(255.0, window.at<cv::Vec3b>(points[i].y, points[i].x)[1] +  dis[i]/sum_distance * 255.0);
            window.at<cv::Vec3b>(points[i].y, points[i].x)[1] = color;
            
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
