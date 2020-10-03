#include "pointsMatcher.h"

// Get the coordinates of the keypoints in the image frame.
vector<Point2f> getKeypointCoordinates(const vector<KeyPoint>& keypoints)
{
    vector<Point2f> coords;
    coords.reserve(keypoints.size());

    for (const auto& kp : keypoints)
    {
        coords.push_back(kp.pt);
    }
    return coords;
}

// Compute the keypoints and descriptors of an image. 
void getImageKeypoints(Mat img, kpt_array &keypoints, Mat &descriptors, string feature_type)
{
    if(feature_type == "SIFT")
    {
        // Initialize detector
        int nfeatures = 0;
        int nOctaveLayers = 2;
        Ptr<SIFT> detector = SIFT::create( nfeatures, nOctaveLayers);
        // Detect keypoints and compute descriptors
        detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    }
    else if(feature_type == "SURF")
    {
        // Initialize detector
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create( minHessian );
        // Detect keypoints and compute descriptors
        detector->detectAndCompute(img, noArray(), keypoints, descriptors);
    }
    else if(feature_type == "FFD")
    {
        // Initialize detector
        Ptr<FastFeatureDetector> ffd = FastFeatureDetector::create();
        // Detect keypoints
        ffd->detect(img, keypoints);
    }
}

// Match the keypoints between two images.
void getImageMatches(Mat descriptors1, Mat descriptors2, vector<vector<DMatch>> &initial_matches, string matcher_type)
{
    // Construct matcher
    Ptr<DescriptorMatcher> matcher;

    if (matcher_type == "bruteforce")
    {
        matcher = BFMatcher::create();
    }
    else if (matcher_type == "flannbased")
    {
        matcher = FlannBasedMatcher::create();
    }
    else
    {
        cout << "ERROR: Unknown matcher type '" << matcher_type << "'\n";
        return;
    }

    // Find the closest two matches
    matcher->knnMatch(descriptors1, descriptors2, initial_matches, 2);
    cout << "INFO: Number of initial matches: " << initial_matches.size() << endl;
}

// Judge match quality based on Lowe's ratio criterion (from SIFT paper) Keep only matches where
// the difference between the first and second match is large enough.
void filterMatches(vector<vector<DMatch>> &initial_matches,
                   vector<DMatch> &filtered_matches,
                   kpt_array &keypoints1,
                   kpt_array &keypoints2,
                   kpt_array &matched_keypoints1,
                   kpt_array &matched_keypoints2)
{
    int idx = 0;

    for (const auto& match : initial_matches)
    {
        if (match[0].distance < 0.8f * match[1].distance)
        {
            filtered_matches.push_back(DMatch(idx, idx, match[0].distance));
            matched_keypoints1.push_back(keypoints1[match[0].queryIdx]);
            matched_keypoints2.push_back(keypoints2[match[0].trainIdx]);
            idx++;
        }
    }

    if (filtered_matches.size() < 4)
    {
        cout << "ERROR: Too few matches!" << endl;
        return;
    }

    cout << "INFO: Number of good matches: " << filtered_matches.size() << endl;
}

// Visualize the matching points between the two images
void visualizeMatches(Mat img1, 
                      Mat img2,
                      vector<DMatch> &filtered_matches,
                      kpt_array &matches1,
                      kpt_array &matches2)
{
    Mat img_out;
    drawMatches(img1, matches1,
                img2, matches2,
                filtered_matches, img_out,
                Scalar::all(-1), Scalar::all(-1)); // Draw only inliers
    imshow("Matches", img_out);
    while (waitKey() != 27);

}

// Visualize the result of the dense optical flow.
void visualizeDenseOF(Mat flowUmat)
{
    Mat flow_parts[2];
    split(flowUmat, flow_parts);
    Mat magnitude, angle, magn_norm;
    cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));
    //build hsv image
    Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cvtColor(hsv8, bgr, COLOR_HSV2BGR);
    imshow("Dense optical flow result", bgr);
    while (waitKey() != 27);
}

// Filter the result of the spare optical flow in order to remove the points which had high error.
void filterOFsparse(vector<uchar> vstatus, 
                    vector<float> verror,
                    vector<Point2f> &original_points,
                    vector<Point2f> &filterd_points,
                    vector<int> &filterd_points_idx)
{
    for (unsigned int i=0; i<vstatus.size(); i++)
    {
        if (vstatus[i] && verror[i] < 12.0)
        {
            // Keep the original index of the point in the
            // optical flow array, for future use
            filterd_points_idx.push_back(i);
            // Keep the feature point itself
            filterd_points.push_back(original_points[i]);
        }
        else
        {
            vstatus[i] = 0; // a bad flow
        }
    }
}

// For the resulting OF points in the right image, find the closest two points in the original right image.
void assignFeatureToPoints(vector<Point2f> &points,                 // Points to be matched
                           vector<Point2f> &features,               // coordinated of the Features to be matched
                           vector<vector<DMatch>> &nearest_feature) // Matched pairs point-feature
{
    Mat points_flat = Mat(points).reshape(1, points.size());
    Mat features_flat = Mat(features).reshape(1, features.size());

    // Brute force matcher for matchin
    Ptr<BFMatcher> matcher = BFMatcher::create();
    matcher->radiusMatch(points_flat, features_flat, nearest_feature, 3.0f);
}

// Match the points between the left and right image.
void getMatchingPoints(vector<vector<DMatch>> &nearest_neighbors,     // Matched pairs point-feature
                       vector<int>     &points_index,          // Indexes of the points regarding the left image
                       vector<DMatch>  &points_matches) //
{
    // Check that the found neighbors are unique (throw away neighbors that are too close together, as they may be confusing)
    set<int>found_points; // for duplicate prevention

    for(int i=0;i<nearest_neighbors.size();i++)
    {
        DMatch _m;
        if(nearest_neighbors[i].size()==1)
        {
            _m = nearest_neighbors[i][0]; // only one neighbor
        }
        else if(nearest_neighbors[i].size()>1)
        {
            // 2 neighbors – check how close they are
            double ratio = nearest_neighbors[i][0].distance / nearest_neighbors[i][1].distance;
            if(ratio < 0.7)
            {
                _m = nearest_neighbors[i][0]; // not too close take the closest (first) one
            }
            else
            { // too close – we cannot tell which is better
                continue; // did not pass ratio test – throw away
            }
        }
        else
        {
            continue; // no neighbors
        }
        // prevent duplicates
        if (found_points.find(_m.trainIdx) == found_points.end())
        {
            // The found neighbor was not yet used: We should match it with the original indexing of the left point
            _m.queryIdx = points_index[_m.queryIdx];
            points_matches.push_back(_m); // add this match
            found_points.insert(_m.trainIdx);
        }
    }
    cout<<"INFO: pruned "<< points_matches.size() <<" / "<<nearest_neighbors.size() <<" matches"<<endl;
}

// Compute the matching points between two images using sparse optical flow. 
// Based on the code provided in: Mastering OpenCV with practical Computer vision projects P.126
void computeSparseOF(Mat img1, Mat img2, vector<Point2f> &imgpts1, vector<Point2f> &imgpts2)
{
    kpt_array keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    getImageKeypoints(img1, keypoints1, descriptors1, "FFD");
    getImageKeypoints(img2, keypoints2, descriptors2, "FFD");

    vector<Point2f> left_points = getKeypointCoordinates(keypoints1);
    vector<Point2f> right_points(left_points.size());

    Mat prevgray, gray;
    // Convert both images to grayscale
    cvtColor(img1, prevgray, COLOR_RGB2GRAY);
    cvtColor(img2, gray, COLOR_RGB2GRAY);

    // Calculate the optical flow field using the lucas-kanade method.
    vector<uchar> vstatus;
    vector<float> verror;
    calcOpticalFlowPyrLK(prevgray, gray, left_points, right_points, vstatus, verror);

    // First, filter out the points with high error
    vector<Point2f> right_points_to_find;
    vector<int> right_points_to_find_back_index;
    filterOFsparse(vstatus, verror, right_points, right_points_to_find, right_points_to_find_back_index);
    vector<Point2f> right_features = getKeypointCoordinates(keypoints2);
    
    // Look around each OF point in the right image for any features that were detected in its area and make a match.
    vector<DMatch> point_feature_matches;
    vector<vector<DMatch>> nearest_neighbors;
    assignFeatureToPoints(right_points_to_find, right_features, nearest_neighbors);
    // Get the matching points between the two images
    getMatchingPoints(nearest_neighbors, right_points_to_find_back_index, point_feature_matches);
    
    for( unsigned int i = 0; i<point_feature_matches.size(); i++ )
    {
        // Save the point coordinates
        imgpts1.push_back(keypoints1[point_feature_matches[i].queryIdx].pt);
        imgpts2.push_back(keypoints2[point_feature_matches[i].trainIdx].pt);

        // Add arrows to the images for visualization of the optical flow
        arrowedLine(img1,	keypoints1[point_feature_matches[i].queryIdx].pt, keypoints2[point_feature_matches[i].trainIdx].pt, CV_RGB(0,255,0), 1);
        arrowedLine(img2,	keypoints1[point_feature_matches[i].queryIdx].pt, keypoints2[point_feature_matches[i].trainIdx].pt, CV_RGB(0,255,0), 1);
    }

    Mat dst;
    hconcat(img1, img2, dst);
    imshow("Optical flow", dst);
    while (waitKey() != 27);
}

// Compute the matching points between two images using dense optical flow.
void computeDenseOF(Mat img1, Mat img2, vector<Point2f> &imgpts1, vector<Point2f> &imgpts2)
{
    kpt_array keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    getImageKeypoints(img1,  keypoints1, descriptors1, "FFD");
    getImageKeypoints(img2, keypoints2, descriptors2, "FFD");
    
    vector<Point2f> left_points = getKeypointCoordinates(keypoints1);
    vector<Point2f>right_points(left_points.size());
    right_points = getKeypointCoordinates(keypoints2);

    Mat mask, prevgray,gray;
    cvtColor(img1,prevgray,COLOR_RGB2GRAY);
    cvtColor(img2,gray,COLOR_RGB2GRAY);

        threshold(prevgray, mask, 50, 255, THRESH_BINARY);
    bitwise_and(prevgray, mask, prevgray);
        threshold(gray, mask, 50, 255, THRESH_BINARY);
    bitwise_and(gray, mask, gray);


    // Calculate the optical flow field: how each left_point moved across the 2 images
    Mat flowUmat;
    calcOpticalFlowFarneback(prevgray,
                             gray,
                             flowUmat,
                             0.5, //pyr_scale
                             4, //levels
                             12, //winsize
                             2, //iterations
                             5, //poly_n
                             1.2, //poly_sigma
                             0);

    visualizeDenseOF(flowUmat);

    //Mat flow_parts[2];
    //split(flowUmat, flow_parts);
    //vector<uint> shape = img1.size();
    int good_points =0;


    Mat flow_parts[2];
    split(flowUmat, flow_parts);
    Mat magnitude, angle, magn_norm;
    cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));
    //build hsv image
    Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cvtColor(hsv8, bgr, COLOR_HSV2BGR);
 
    
    Mat im_thresh_gray, im_gray;
    cvtColor(bgr,im_gray,COLOR_RGB2GRAY);
    threshold(gray, mask, 50, 255, THRESH_BINARY);
    bitwise_and(gray, mask, im_thresh_gray);
        imshow("Dense optical flow result", im_thresh_gray);
    while (waitKey() != 27);


    for(float i=0; i<480; i++)
    {
        for(float j=0; j<640; j++)
        {
            float delta_x =  flow_parts[0].at<float>(i,j);
            float delta_y =  flow_parts[1].at<float>(i,j);

            float distance = sqrt(delta_x*delta_x + delta_y*delta_y);
            
            Vec3b pixel_color = hsv.at<Vec3b>(i,j);

            //cout << delta_x << " , " << delta_y << endl;
            //if(mask.at<float>(i,j))
            if(distance < 120 && distance > 0.01)
            {
                imgpts1.push_back(Point2f(int(i), int(j)));
                imgpts2.push_back(Point2f(int(i + delta_x) , int(j + delta_y)));
                good_points++;
            }
        }
    }

    cout << "INFO: Filtered matches " << good_points << " / " << 480*640 << endl;
}

// Constructor of the class
pointsMatcher::pointsMatcher(Mat img1, Mat img2, string selected_matcher)
{
    img1.copyTo(_img1);
    img2.copyTo(_img2);

    _selected_matcher = selected_matcher;
}

void pointsMatcher::computeMatches(vector<Point2f> &imgpts1, vector<Point2f> &imgpts2, string matcher_type)
{
    if(_selected_matcher == "SIFT" ||  _selected_matcher == "SURF")
    {
        kpt_array keypoints1, keypoints2;
        Mat descriptors1, descriptors2;

        getImageKeypoints(_img1, keypoints1, descriptors1, _selected_matcher);
        getImageKeypoints(_img2, keypoints2, descriptors2, _selected_matcher);

        // Find 2 nearest correspondences for each descriptor
        vector<vector<DMatch>> initial_matches;
        getImageMatches(descriptors1, descriptors2, initial_matches, matcher_type);
        
        vector<DMatch> filtered_matches;
        kpt_array filtered_matches_1;
        kpt_array filtered_matches_2;
        filterMatches(initial_matches, filtered_matches, keypoints1, keypoints2, filtered_matches_1, filtered_matches_2);

        imgpts1 = getKeypointCoordinates(filtered_matches_1);
        imgpts2 = getKeypointCoordinates(filtered_matches_2);

        visualizeMatches(_img1, _img2, filtered_matches, filtered_matches_1, filtered_matches_2);
    }
    else if(_selected_matcher == "OF_SPARSE")
    {
        computeSparseOF(_img1, _img2, imgpts1, imgpts2);
    }
    else if(_selected_matcher == "OF_DENSE")
    {
        computeDenseOF(_img1, _img2, imgpts1, imgpts2);
    }
    else
    {
        cout << "ERROR: Invalid matcher_type" << endl;
    }
    
}


