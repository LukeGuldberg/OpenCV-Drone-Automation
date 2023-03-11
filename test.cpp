#include <opencv2/core/types_c.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

#include "hashpoints.h"

constexpr int maxCorners = 40;
constexpr double qualityLevel = 0.01;
constexpr double minDistance = 20.;
constexpr int blockSize = 3;
constexpr bool useHarrisDetector = false;
constexpr double k = 0.04;

std::vector<cv::Scalar> generate_random_colors();
void refresh_points(cv::Mat mask, cv::Mat grayMat,
                    std::vector<cv::Point2f>& corners,
                    std::vector<cv::Point2f>& oldCorners);
std::vector<cv::Point2f> find_good_points(cv::Mat oldGray, cv::Mat grayMat,
                                          std::vector<cv::Point2f> oldCorners,
                                          cv::Mat frame,
                                          std::vector<cv::Scalar> colors,
                                          HashTable& table, bool& refresh);
void print_table(HashTable table, std::ofstream& outfile);

int main() {
    std::ofstream outfile("data.txt");

    cv::Mat image1 =
        cv::imread("C:/Users/lukeg/code/opencv-msys/images/img1.jpg");
    cv::Mat image2 =
        cv::imread("C:/Users/lukeg/code/opencv-msys/images/img2.jpg");
    std::vector<cv::Mat> images = {image1, image2};

    std::vector<cv::Scalar> colors = generate_random_colors();
    HashTable table;

    cv::Mat oldGray, oldFrame, mask;
    oldFrame = images.at(0);
    cvtColor(oldFrame, oldGray, cv::COLOR_BGR2GRAY);

    bool refresh = true;
    std::vector<cv::Point2f> oldCorners;  // corners for current frame

    int index = 1;
    while (true) {
        std::vector<cv::Point2f> corners;
        cv::Mat frame, grayMat;
        frame = images.at(index).clone();

        cv::cvtColor(frame, grayMat, cv::COLOR_BGR2GRAY);

        if (refresh) {
            refresh_points(mask, grayMat, corners,
                           oldCorners);  // change position of if statement
                                         // testing true or false
            refresh = false;
        }

        // Calulating the optical flow - finding GOOD POINTS
        std::vector<cv::Point2f> good_new = find_good_points(
            oldGray, grayMat, oldCorners, frame, colors, table, refresh);

        oldGray = grayMat.clone();  // save the previous frame
        oldCorners = good_new;      // save the previous set
        cv::imshow("Tello", frame);
        cv::waitKey(0);
        if (cv::waitKey(1) == 27) {  // ESC
            break;
        }
        index = (index + 1) % images.size();

        print_table(table, outfile);
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

void refresh_points(cv::Mat mask, cv::Mat grayMat,
                    std::vector<cv::Point2f>& corners,
                    std::vector<cv::Point2f>& oldCorners) {
    cv::goodFeaturesToTrack(grayMat, corners, maxCorners, qualityLevel,
                            minDistance, mask, blockSize, useHarrisDetector, k);
    for (auto a : corners) {
        oldCorners.push_back(a);
    }
}

std::vector<cv::Point2f> find_good_points(cv::Mat oldGray, cv::Mat grayMat,
                                          std::vector<cv::Point2f> oldCorners,
                                          cv::Mat frame,
                                          std::vector<cv::Scalar> colors,
                                          HashTable& table, bool& refresh) {
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2f> corners2;

    std::vector<uchar> status;
    std::vector<float> err;
    cv::TermCriteria criteria = cv::TermCriteria(
        (cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);

    // USE THE SET/VECTOR THAT ADDED NEW GOODFEATURES
    cv::calcOpticalFlowPyrLK(oldGray, grayMat, oldCorners, corners, status, err,
                             cv::Size(15, 15), 2, criteria);
    // cv::calcOpticalFlowPyrLK(grayMat, oldGray, corners, corners2, status,
    // err,
    //                          cv::Size(15, 15), 2, criteria);

    std::vector<cv::Point2f> good_new;
    for (uint i = 0; i < oldCorners.size(); i++) {
        // Select good points
        // float d = abs(oldCorners - corners2).reshape(-1, 2).max(-1);
        // bool good = d < 1;
        if (status[i] == 1) {
            good_new.push_back(corners.at(i));
            // draw the tracks
            line(frame, corners.at(i), oldCorners.at(i), colors.at(i), 2);
            circle(frame, corners.at(i), 5, colors.at(i), -1);
            if (table.search(oldCorners.at(
                    i))) {  // if it already exists, erase and replace
                table.erase_and_replace(oldCorners.at(i), corners.at(i));
            } else {  // if it doesnt exist yet, just insert
                table.insert(corners.at(i));
            }
            // } else if (table.search(oldCorners.at(
            //                i))) {  // if good point goes away and it was
            //                previously
            //                        // inserted to table, erase
        } else {
            table.erase(oldCorners.at(i));
        }
    }

    if (status.size() < 20) {
        refresh = true;
    }
    return good_new;
}

void print_table(HashTable table, std::ofstream& outfile) {
    outfile << "\nNEW FRAME!!!\n";
    for (auto t : table.table) {
        outfile << "key: (" << t.first.x << ", " << t.first.y << ")\tvalue:\n ";
        for (auto y : t.second) {
            outfile << "(" << y.x << ", " << y.y << ")->";
        }
        outfile << "\n";
    }
}
