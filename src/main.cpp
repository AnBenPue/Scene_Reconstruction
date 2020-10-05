#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <reconstructor.h>
#include <pointsMatcher.h>


/* Reference: https://answers.opencv.org/question/42770/how-to-read-matrix-from-text-file-to-mat-in-opencv/?answer=42806
   To read data from a text file. 
   filename is the name of the text file
   rows and cols show dimensions of the matrix written in the text fil
*/
Mat ReadMatFromTxt(string filename, int rows,int cols)
{
    double m;
    Mat out = Mat::zeros(rows, cols, CV_64FC1);//Matrix to store values

    ifstream fileStream(filename);
    int cnt = 0;//index starts from 0
    while (fileStream >> m)
    {
        int temprow = cnt / cols;
        int tempcol = cnt % cols;
        out.at<double>(temprow, tempcol) = m;
        cnt++;
    }
    return out;
}

int main(int argc, char* argv[])
{
    CommandLineParser parser(argc, argv,
                             "{help     |                  | print this message}"
                             "{@image1  | ./img1.jpg       | image1 path}"
                             "{@image2  | ./img2.jpg       | image2 path}"
                             "{K        | ./path/to/K.txt  | intrisic parameters matrix path}"
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
    string K_path = parser.get<string>("K");
    Mat K = ReadMatFromTxt(K_path, 3, 3);

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






