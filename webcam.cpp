#include <opencv2/core/types_c.h>

#include <iomanip>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>  //set
#include <queue>
#include <set>
#include <vector>
namespace std {
template <>
struct hash<cv::Point2f> {
    void hash_combine(std::size_t& seed, int value) const {
        seed ^= hash<int>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    std::size_t operator()(const cv::Point2f p) const {
        std::size_t seed = 0;
        hash_combine(seed, p.x);
        hash_combine(seed, p.y);
    }
};
}  // namespace std

constexpr int maxCorners = 10;
constexpr double qualityLevel = 0.01;
constexpr double minDistance = 20.;
constexpr int blockSize = 3;
constexpr bool useHarrisDetector = false;
constexpr double k = 0.04;

std::vector<cv::Scalar> generate_random_colors();
std::vector<cv::Point2f> find_good_points(cv::Mat oldGray, cv::Mat grayMat,
                                          std::vector<cv::Point2f> oldCorners,
                                          std::vector<cv::Point2f> corners,
                                          cv::Mat frame,
                                          std::vector<cv::Scalar> colors);

int main() {
    cv::VideoCapture capture{0};
    if (!capture.isOpened()) {
        std::cerr << "Unable to open capture stream\n";
    }

    // generaterandomcolors()
    std::vector<cv::Scalar> colors = generate_random_colors();
    cv::Mat oldGray, oldFrame,
        mask;  // used in calcopticalflow , last frames imgs
    std::vector<cv::Point2f> oldCorners;  // corners for the last frame
    std::hash<cv::Point2f> oldSet;

    // Take first frame and find corners in it
    capture >> oldFrame;
    cvtColor(oldFrame, oldGray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(oldGray, oldCorners, 100, 0.3, 7, cv::Mat(), 7, false,
                        0.04);

    while (true) {
        std::vector<cv::Point2f> corners;  // corners for current frame
        cv::Mat frame, grayMat;

        if (!capture.read(frame)) {
            std::cerr << "Read blank frame\n";
            continue;
        }

        cv::cvtColor(frame, grayMat, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(grayMat, corners, maxCorners, qualityLevel,
                                minDistance, mask, blockSize, useHarrisDetector,
                                k);

        // ADD GOODFEATURES (CORNERS) TO SET
        // for (int i = 0; i < corners.size(); ++i) {  // add good features
        // to set
        //     oldSet(corners[i]);
        // }
        // // PUT SET BACK INTO A VECTOR so it can be used in
        // find_good_points() for (auto it = oldSet.begin(); it !=
        // oldSet.end(); ++it) {
        //     // oldcorners.push_back(it.value())
        // }

        // Calulating the optical flow - finding GOOD POINTS
        std::vector<cv::Point2f> good_new = find_good_points(
            oldGray, grayMat, oldCorners, corners, frame, colors);

        std::hash<cv::Point2f> set;
        for (int i = 0; i < good_new.size(); ++i) {  // add good_new to set
            set(good_new[i]);
            // this is the new x,y pairs that will be looked at next time
            // in addition to the next call of goodfeaturestotrack()
        }
        cv::imshow("Tello", frame);
        oldGray = grayMat.clone();
        oldCorners = good_new;
        oldSet = set;
        // SAVE SET
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
                                          std::vector<cv::Point2f> corners,
                                          cv::Mat frame,
                                          std::vector<cv::Scalar> colors) {
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