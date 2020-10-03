#include <iostream>
#include <opencv2/core.hpp>
#include <reconstructor.h>
#include <pointsMatcher.h>

int main(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv,
                             "{help     |                  | print this message}"
                             "{@image1  | ./img1.jpg       | image1 path}"
                             "{@image2  | ./img2.jpg       | image2 path}"
                             "{features | sift             | feature type , SIFT, SURF, OF_SPARSE, OF_DENSE}"
                             "{matcher  | bruteforce       | matcher type}"
                             );

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Load images
    string filepath1 = parser.get<string>("@image1");
    string filepath2 = parser.get<string>("@image2");
    Mat img1 = imread(filepath1);
    Mat img2 = imread(filepath2);

    if (img1.empty())
    {
        cout << "ERROR: Input image 1 not found at '" << filepath1 << "'\n";
        return 1;
    }

    if (img2.empty())
    {
        cout << "ERROR: Input image 2 not found at '" << filepath2 << "'\n";
        return 1;
    }

    // Get the matching points between the two images
    string features_type = parser.get<string>("features");
    cout << "INFO: Feature type: " << features_type << endl;

    pointsMatcher M =pointsMatcher(img1, img2, features_type);
    vector<Point2f> imgpts1, imgpts2;
    M.computeMatches(imgpts1, imgpts2, "bruteforce");

    // Reconstruct the scene
    // Intrisic parameters of the camera
    Mat K = (Mat_<double>(3,3) << 1520.400000, 0.000000, 246.87, 0.000000, 1525.900000, 302.320000, 0.000000, 0.000000, 1.000000);

    reconstructor R = reconstructor(imgpts1, imgpts2, K);

    // Depending on the feature, the ransac reprojection threshold need to be adjusted.
    if(features_type == "SIFT")
    {
        R.getEssentialMatrix(0.1);
    }
    else if(features_type == "SURF")
    {
        R.getEssentialMatrix(0.1);
    }
    else if(features_type == "OF_SPARSE")
    {
        R.getEssentialMatrix(1.0);
    }
    else if(features_type == "OF_DENSE")
    {
        R.getEssentialMatrix(0.1);
    }
    else
    {
        return 0;
    }

    // Estimate the camera motion and build and reconstruct the scene.
    R.getCameraMatrices();
    R.getPointCloud();

    // Run the point cloud visualization
    system("python3 ../visualization.py");
    
    return 0;
}






