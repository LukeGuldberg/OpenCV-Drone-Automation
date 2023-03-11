#include <opencv2/core/types_c.h>

#include <functional>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <unordered_map>
namespace std {
template <>
struct hash<cv::Point2f> {
    // void hash_combine(std::size_t& seed, int value) const {
    //     seed ^= hash<float>{}(value) + 0x9e3779b9 + (seed << 6) + (seed
    //     >> 2);
    // }
    std::size_t operator()(const cv::Point2f& v) const {
        std::size_t x = std::hash<float>{}(v.x);
        std::size_t y = std::hash<float>{}(v.y);
        return x ^ (y << 1);
    }
};
}  // namespace std

class HashTable {
public:
    void insert(const cv::Point2f point) {
        table[point];
    }
    // USE COUNT - returns 0 or 1
    // when a good-point disappears
    void erase(const cv::Point2f point) {
        table.erase(point);
        std::cout << "Erased " << point << "\n";
    }

    // when the good-point moves
    void erase_and_replace(cv::Point2f old_point, cv::Point2f new_point) {
        std::vector<cv::Point2f> points = table[old_point];
        points.push_back(old_point);
        table[new_point] = points;
        table.erase(old_point);
    }

    bool search(cv::Point2f point) {
        return table.find(point) != table.end();
    }

    std::vector<cv::Point2f> lookup(cv::Point2f point) {
        return table[point];
    }

    std::unordered_map<cv::Point2f, std::vector<cv::Point2f>> table;

private:
    std::hash<cv::Point2f> hash;
};
