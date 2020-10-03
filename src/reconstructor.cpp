#include "reconstructor.h"

// Code extracted from the book: Mastering OpenCV with Practical Computer Vision Projects. P.132
bool CheckCoherentRotation(cv::Mat_<double>& R)
{
    if(fabsf(determinant(R))-1.0 > 1e-07)
    {
        std::cout << "det(R) != +-1.0, this is not a rotation matrix"<< std::endl;
        return false;
    }
    return true;
}

// Code extracted from the book: Mastering OpenCV with Practical Computer Vision Projects. P.134
Mat_<double> LinearLSTriangulation(Point3d u,//homogenous image point (u,v,1)
                                   Matx34d P,//camera 1 matrix
                                   Point3d u1,//homogenous image point in 2nd camera
                                   Matx34d P1//camera 2 matrix
                                   )
{
    //build A matrix
    Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),    u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),    u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1), u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1), u1.y*P1(2,2)-P1(1,2)
              );
    //build B vector
    Matx41d B(-(u.x*P(2,3)-P(0,3)),
              -(u.y*P(2,3)-P(1,3)),
              -(u1.x*P1(2,3)-P1(0,3)),
              -(u1.y*P1(2,3)-P1(1,3)));
    //solve for X
    Mat_<double> X;
    solve(A,B,X,cv::DECOMP_SVD);
    Mat X_h = (Mat_<double>(4,1) << X(0,0), X(1,0), X(2,0), 1.000000);

    return X_h;
}

// Code extracted from the book: Mastering OpenCV with Practical Computer Vision Projects. P.134
double TriangulatePoints(const std::vector<Point2f> &pt_set1,
                         const std::vector<Point2f> &pt_set2,
                         const Mat &Kinv,
                         const Matx34d &P,
                         const Matx34d &P1,
                         std::vector<Point3d> &pointcloud)
{
    std::vector<double> reproj_error;
    int pts_size = pt_set1.size();
    for (unsigned int i=0; i<pts_size; i++)
    {
        //convert to normalized homogeneous coordinates
        Point2f kp = pt_set1[i];
        Point3d u(kp.x, kp.y,1.0);
        
        Mat_<double> um = Kinv * Mat_<double>(u);
        u = um.at<Point3d>(0);

        Point2f kp1 = pt_set2[i];
        Point3d u1(kp1.x,kp1.y,1.0);
        Mat_<double> um1 = Kinv * Mat_<double>(u1);

        u1 = um1.at<Point3d>(0);
        //triangulate
        Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
        //calculate reprojection error

        Mat K;
        cv::invert(Kinv, K);

        Mat_<double> xPt_img = K * Mat(P1) * X;
        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
        reproj_error.push_back(norm(xPt_img_ - kp1));
        //store 3D point
        pointcloud.push_back(Point3d(X(0),X(1),X(2)));
    }
    
    //return mean reprojection error
    Scalar me = mean(reproj_error);
    return me[0];
}

reconstructor::reconstructor(std::vector<Point2f> imgpts1, std::vector<Point2f> imgpts2, Mat K)
{
    _imgpts1 = imgpts1;
    _imgpts2 = imgpts2;
    K.copyTo(_K);
}

// Code extracted from the book: Mastering OpenCV with Practical Computer Vision Projects. P.130
void reconstructor::getEssentialMatrix(float ransacReprojThreshold)
{
    Mat F = cv::findFundamentalMat(_imgpts1, _imgpts2, cv::FM_RANSAC, ransacReprojThreshold, 0.99, 10000);
    _E = _K.t() * F * _K; //according to HZ (9.12)
}

// Code extracted from the book: Mastering OpenCV with Practical Computer Vision Projects. P.13
void reconstructor::getCameraMatrices()
{
    SVD svd(_E);
    Matx33d W(0,-1, 0,//HZ 9.13
              1, 0, 0,
              0, 0, 1);
    Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19

    if (!CheckCoherentRotation(R))
    {
        cout<<"resulting rotation is not coherent\n";
        return;
    }

    Mat_<double> t = svd.u.col(2); //u3
    Matx34d P1( R(0,0),R(0,1), R(0,2), t(0),
                R(1,0),R(1,1), R(1,2), t(1),
                R(2,0),R(2,1), R(2,2), t(2));

    Matx34d P( 1,0,0,0,
               0,1,0,0,
               0,0,1,0);
    _P = P;
    _P1 = P1;
}

void savePointCloudToFile(std::string filename, std::vector<Point3d> &point_cloud)
{
    std::ofstream output_file(filename);
    std::ostream_iterator<Point3d> output_iterator(output_file, "\n");
    std::copy(point_cloud.begin(), point_cloud.end(), output_iterator);
}

void reconstructor::getPointCloud()
{
    Mat Kinv;
    cv::invert(_K, Kinv);
    std::vector<Point3d> cloud_result;
    double error = TriangulatePoints(_imgpts1,
                                     _imgpts2,
                                     Kinv,
                                     _P,
                                     _P1,
                                     cloud_result);

    std::cout << "INFO: error = " << error << std::endl;
    savePointCloudToFile("./scene_pointcloud.txt", cloud_result);
}
