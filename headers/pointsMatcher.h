#ifndef MATCHER_H
#define MATCHER_H

#include <set>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/video/tracking.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using kpt_array = vector<KeyPoint> ;

// Find the matching points between two images
class pointsMatcher
{
private:
    Mat _img1;
    Mat _img2;
    string _selected_matcher;
public:
    pointsMatcher(Mat img1, Mat img2, string selected_matcher);
    void computeMatches(vector<Point2f> &imgpts1, vector<Point2f> &imgpts2, string matcher_type);
};

#endif // MATCHER_H
