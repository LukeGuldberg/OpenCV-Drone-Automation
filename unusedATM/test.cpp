#include <iomanip>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <queue>
#include <string>
#include <vector>

using namespace cv;

int main() {
    cv::VideoCapture capture{0};
    if (!capture.isOpened()) {
        std::cerr << "Unable to open capture stream\n";
        return -1;
    }

    int maxCorners = 10;
    double qualityLevel = 0.01;
    double minDistance = 20.;
    cv::Mat mask;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    std::queue<cv::Point2f> queue;

    while (true) {
        std::vector<cv::Point2f> corners;
        cv::Mat frame;
        // std::vector<cv::KeyPoint> keypoints;
        if (!capture.read(frame)) {
            std::cerr << "Read blank frame\n";
            continue;
        }

        // image processing code here

        // greyscale
        cv::Mat greyMat;
        cv::cvtColor(frame, greyMat, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(greyMat, corners, maxCorners, qualityLevel,
                                minDistance, mask, blockSize, useHarrisDetector,
                                k);
        for (size_t i = 0; i < corners.size(); i++) {  // draw circles
            cv::circle(greyMat, corners[i], 3, cv::Scalar(255.),
                       -1);  // circles on greyscale
            cv::circle(frame, corners[i], 3, cv::Scalar(255.),
                       -1);  // cirlces on BGR
            queue.push(corners[i]);
            // std::cout << corners[i] << "\n";
        }
        std::cout << queue.front() << "  ";
        queue.pop();
        while (!queue.empty()) {
            std::cout << std::setw(20) << queue.front() << "  ";
            queue.pop();
        }
        std::cout << "\n";
        // look up distortion/camera calibration

        // implement FAST algorithm and find key points
        // cv::drawKeypoints
        cv::imshow("Tello", frame);
        cv::imshow("Tello_Grey", greyMat);
        if (cv::waitKey(1) == 27) {  // ESC
            break;
        }
    }
}