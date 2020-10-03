#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include <iostream>
#include <fstream>
#include <iterator>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
 
using namespace cv;
using namespace std;

// Reconstruct the point cloud based on the matching points in two images
class reconstructor
{
private:
    std::vector<Point2f> _imgpts1;
    std::vector<Point2f> _imgpts2;
    Mat _K;
    Mat_<double> _E;
    Matx34d _P;
    Matx34d _P1;

public:
    reconstructor(std::vector<Point2f> imgpts1, std::vector<Point2f> imgpts2, Mat K);
    void getEssentialMatrix(float ransacReprojThreshold);
    void getCameraMatrices();
    void getPointCloud();
};

#endif // RECONSTRUCTOR_H
