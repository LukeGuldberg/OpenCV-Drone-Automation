#include <opencv2/core/types_c.h>

#include <iomanip>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <queue>
#include <set>
#include <vector>
struct Point2fComparator {
    bool operator()(const cv::Point2f& p1, const cv::Point2f& p2) const {
        return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
    }
};

constexpr int maxCorners = 10;
constexpr double qualityLevel = 0.01;
constexpr double minDistance = 20.;
constexpr int blockSize = 3;
constexpr bool useHarrisDetector = false;
constexpr double k = 0.04;

std::vector<cv::Scalar> generate_random_colors();
std::vector<cv::Point2f> find_good_points(cv::Mat oldGray, cv::Mat grayMat,
                                          std::vector<cv::Point2f> oldCorners,
                                          cv::Mat frame,
                                          std::vector<cv::Scalar> colors);

int main() {
    cv::VideoCapture capture{0};
    if (!capture.isOpened()) {
        std::cerr << "Unable to open capture stream\n";
    }
    std::vector<cv::Scalar> colors = generate_random_colors();

    cv::Mat oldGray, oldFrame, mask;
    std::set<cv::Point2f, Point2fComparator> oldSet;
    capture >> oldFrame;
    cvtColor(oldFrame, oldGray, cv::COLOR_BGR2GRAY);
    while (true) {
        std::vector<cv::Point2f> oldCorners;  // corners for current frame
        cv::Mat frame, grayMat;

        if (!capture.read(frame)) {
            std::cerr << "Read blank frame\n";
            continue;
        }

        cv::cvtColor(frame, grayMat, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(grayMat, oldCorners, maxCorners, qualityLevel,
                                minDistance, mask, blockSize, useHarrisDetector,
                                k);

        for (auto c : oldCorners) {  // adds the new good features to the last
            oldSet.insert(c);        // frames good_new that was returned
        }
        for (const auto& s : oldSet) {  // gets put back into a vector to be
            oldCorners.push_back(s);    // handled by find_good_points
        }

        // Calulating the optical flow - finding GOOD POINTS

        std::vector<cv::Point2f> good_new =
            find_good_points(oldGray, grayMat, oldCorners, frame, colors);

        std::set<cv::Point2f, Point2fComparator> set;
        for (int i = 0; i < 20; ++i) {  // add good_new to set
            set.insert(good_new[i]);
            // this is the new x,y pairs that will be looked at next time
            // in addition to the next call of goodfeaturestotrack()
        }

        oldGray = grayMat.clone();  // save the previous frame
        oldSet = set;               // save the previous set
        cv::imshow("Tello", frame);
        if (cv::waitKey(1) == 27) {  // ESC
            break;
        }
    }
}

std::vector<cv::Scalar> generate_random_colors() {
    std::vector<cv::Scalar> colors;
    cv::RNG rng;
    for (int i = 0; i < 100; i++) {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r, g, b));
    }
    return colors;
}

std::vector<cv::Point2f> find_good_points(cv::Mat oldGray, cv::Mat grayMat,
                                          std::vector<cv::Point2f> oldCorners,
                                          cv::Mat frame,
                                          std::vector<cv::Scalar> colors) {
    std::vector<cv::Point2f> corners;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Mat mask;
    cv::TermCriteria criteria = cv::TermCriteria(
        (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
    // USE THE SET/VECTOR THAT ADDED NEW GOODFEATURES
    cv::calcOpticalFlowPyrLK(oldGray, grayMat, oldCorners, corners, status, err,
                             cv::Size(15, 15), 2, criteria);
    // cv::calcOpticalFlowPyrLK(oldGray, grayMat, corners, #####newval,
    // status,
    //                          err, Size(15, 15), 2, criteria);
    std::vector<cv::Point2f> good_new;
    for (uint i = 0; i < oldCorners.size(); i++) {
        // Select good points
        if (status[i] == 1) {
            good_new.push_back(corners[i]);
            // draw the tracks
            line(mask, corners[i], oldCorners[i], colors[i], 2);
            circle(frame, corners[i], 5, colors[i], -1);
        }
    }
    return good_new;
}