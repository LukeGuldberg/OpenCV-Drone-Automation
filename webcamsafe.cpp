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
std::set<cv::Point2f> convertToSet(std::vector<cv::Point2f> v);

int main() {
    cv::VideoCapture capture{0};
    if (!capture.isOpened()) {
        std::cerr << "Unable to open capture stream\n";
        return -1;
    }

    // generaterandomcolors()
    std::vector<cv::Scalar> colors;
    cv::RNG rng;
    for (int i = 0; i < 100; i++) {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r, g, b));
    }

    int maxCorners = 10;
    double qualityLevel = 0.01;
    double minDistance = 20.;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    cv::Mat mask;
    std::queue<cv::Point2f> queue;
    cv::Mat oldGray, oldFrame;  // used in calcopticalflow , last frames imgs
    std::vector<cv::Point2f> oldCorners;  // corners for the last frame
    std::set<cv::Point2f> points;
    std::hash<cv::Point2f> set;

    // Take first frame and find corners in it
    capture >> oldFrame;
    cvtColor(oldFrame, oldGray, cv::COLOR_BGR2GRAY);
    goodFeaturesToTrack(oldGray, oldCorners, 100, 0.3, 7, cv::Mat(), 7, false,
                        0.04);

    // Create a mask image for drawing purposes
    // Mat mask = Mat::zeros(oldFrame.size(), oldFrame.type());

    while (true) {
        std::vector<cv::Point2f> corners;  // corners for current frame
        cv::Mat frame;
        cv::Mat grayMat;  // current greyscale

        if (!capture.read(frame)) {
            std::cerr << "Read blank frame\n";
            continue;
        }

        // greyscale

        cv::cvtColor(frame, grayMat, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(grayMat, corners, maxCorners, qualityLevel,
                                minDistance, mask, blockSize, useHarrisDetector,
                                k);
        for (int i = 0; i < corners.size(); ++i) {
            set(corners[i]);
        }
        // ADD GOODFEATURES (CORNERS) TO SET
        // PUT SET BACK INTO A VECTOR

        // for (size_t i = 0; i < corners.size(); i++) {  // draw circles
        // cv::circle(greyMat, corners[i], 3, cv::Scalar(255.),
        //            -1);  // circles on greyscale
        // cv::circle(frame, corners[i], 3, cv::Scalar(255.),
        //            -1);          // cirlces on BGR
        // queue.push(corners[i]);  // store in queue
        // }
        // std::cout << queue.front() << "  ";
        // queue.pop();
        // while (!queue.empty()) {  // print queue
        //     std::cout << std::setw(20) << queue.front() << "  ";
        //     queue.pop();
        // }
        // std::cout << "\n";

        // Calulating the optical flow - finding GOOD POINTS
        std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria(
            (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        // USE THE SET/VECTOR THAT ADDED NEW GOODFEATURES
        cv::calcOpticalFlowPyrLK(oldGray, grayMat, oldCorners, corners, status,
                                 err, cv::Size(15, 15), 2, criteria);
        // cv::calcOpticalFlowPyrLK(oldGray, grayMat, corners, #####newval,
        // status,
        //                          err, Size(15, 15), 2, criteria);
        std::vector<cv::Point2f> good_new;
        for (uint i = 0; i < oldCorners.size(); i++) {
            // Select good points
            if (status[i] == 1) {
                good_new.push_back(corners[i]);
                set(corners[i]);
                // draw the tracks
                line(mask, corners[i], oldCorners[i], colors[i], 2);
                circle(frame, corners[i], 5, colors[i], -1);
            }
        }

        // look up distortion/camera calibration
        // implement FAST algorithm and find key points
        // cv::drawKeypoints
        // cv::Mat img;
        // add(grayMat, mask, img);
        cv::imshow("Tello", frame);
        // cv::imshow("Tello_Grey", grayMat);
        oldGray = grayMat.clone();
        oldCorners = good_new;
        // SAVE SET
        if (cv::waitKey(1) == 27) {  // ESC
            break;
        }
    }
}

std::set<cv::Point2f> convertToSet(const std::vector<cv::Point2f> v) {
    // Declaring the set
    std::set<cv::Point2f> s;

    // Traverse the Vector
    // for (int i = 0; i < v.size(); i++) {
    //     // Insert each element
    //     // into the Set
    //     s.insert(v[i]);
    // }

    // for (const auto x : v) {
    //     // Insert each element
    //     // into the Set
    //     s.emplace(x);

    // }

    // std::set<cv::Point2f> s{v.begin(), v.end()};

    // Return the resultant Set
    return s;
}