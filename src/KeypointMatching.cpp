//How to build? g++ ORB.cpp -o ORB `pkg-config --cflags --libs opencv`
//How to call the Function: ./ORB ../../data/1.png ../../data/2.png
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

//#include <ceres/ceres.h>
//#include <ceres/rotation.h>

using namespace std;
using namespace cv;

/*
void write2File(string output,vector <int> Frames,vector<KeyPoint>keypoints_1,vector<KeyPoint>keypoints_2,vector<DMatch> good_matches,int num_cameras)
{
  ofstream outfile(output);

  outfile<<num_cameras<<" "<<keypoints_1.size()<<" "<< num_cameras*good_matches.size()<<" "<<endl;

  vector <Point2f> imgpts1, imgpts2;
  for (vector<DMatch>::size_type i=0;i<good_matches.size();i++)
  {
    //queryIdx is the left image, Frame[0]
    //trainIdx is the right image, Frame[1]
    imgpts1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
    imgpts2.push_back(keypoints_2[good_matches[i].trainIdx].pt);
    cout << "Good Indices:" << imgpts1[i] << i << imgpts2[i] << endl;
    cout << good_matches[i].queryIdx << "QueryIndex" << endl;
    cout << good_matches[i].trainIdx << "trainIndex" << endl;
    outfile << Frames[0]<<" "<<i<<"\t"<<keypoints_1[good_matches[i].queryIdx].pt.x<<" "<<keypoints_1[good_matches[i].queryIdx].pt.y<<endl;
    outfile << Frames[1]<<" "<<i<<"\t"<<keypoints_2[good_matches[i].trainIdx].pt.x<<" "<<keypoints_2[good_matches[i].trainIdx].pt.y<<endl;
  }
  outfile.close();
}

void BundleAdjustment()
{
   //ceres::Solver::Summary summary;
}*/

int main (int argc, char** argv)
{

    if ( argc != 5 )
    {
        cout<<"usage: feature_extraction img1 img2 img3 output"<<endl;
        return 1;
    }

    //TODO: One more for Loop to get pair wise images
    //perform keypoint extraction and then bundle adjustment
    //Loop over the data folder build the image file inde
    //Build a keyFrame name. Each image is a KeyFrame Name
//    vector<cv::String> fn;
//    glob("../../data/*.png", fn, false);
//    vector<Mat> images;
//    vector <int> Frames;
/*    size_t num_cameras = fn.size(); //number of png files in images folder
    for (size_t i=0;i<num_cameras;i++)
    {
      Frames.push_back(i);
      cout << "image names:" << fn[i] << endl;
      cout << "Frame names:" << Frames[i] << endl;
    }*/
    Mat img_1 = imread(argv[1],cv::IMREAD_COLOR);
    Mat img_2 = imread(argv[2],cv::IMREAD_COLOR);
    Mat img_3 = imread(argv[3],cv::IMREAD_COLOR);

    //Vector Keypoints, keypoints
    vector<KeyPoint> keypoints_1, keypoints_2, keypoints_3;
    Mat descriptors_1, descriptors_2, descriptors_3;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1,keypoints_1);
    cout << "Size of keypoints1:" << keypoints_1.size() << endl;
    detector->detect(img_2,keypoints_2);
    cout << "Size of Keypoints2:" << keypoints_2.size() << endl;
    detector->detect(img_3,keypoints_3);
    cout << "Size of Keypoints3:" << keypoints_3.size() << endl;

    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);
    descriptor->compute(img_3,keypoints_3,descriptors_3);

    //Mat outimg1;
    //drawKeypoints(img_1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    //imshow("original img",outimg1);

    vector<DMatch> matches_1; //stores the matches descriptors for image pair 1,3
    vector<DMatch> matches_2; //stores the matches descriptors for image pair 2,3
    matcher->match(descriptors_3, descriptors_1, matches_1);
    matcher->match(descriptors_3, descriptors_2, matches_2);

    /*
    double min_dist=10000, max_dist=0;

    for (int i=0;i<descriptors_3.rows;i++)
    {
        double dist = matches[i].distance;
        if (dist<min_dist) min_dist=dist;
        if (dist>max_dist) max_dist=dist;
    }

    cout << "-- Max dist : %f \n" << max_dist << endl;
    cout << "-- Min dist : %f \n"  << min_dist << endl;
    */


    std::stringstream out;

    int good_match_id = 0;
    for (int i=0;i<descriptors_3.rows;i++)
    {
        if (matches_1[i].distance<=20.0 and matches_2[i].distance<=20.0)
        {
            out << "0 " << good_match_id << " " << (keypoints_1[matches_1[i].trainIdx].pt.x / 960.0 - 1.0) << " " << (keypoints_1[matches_1[i].trainIdx].pt.y / 960.0 - 1.0) << endl;
            out << "1 " << good_match_id << " " << (keypoints_2[matches_2[i].trainIdx].pt.x / 960.0 - 1.0) << " " << (keypoints_2[matches_2[i].trainIdx].pt.y / 960.0 - 1.0) << endl;
            out << "2 " << good_match_id << " " << (keypoints_3[i].pt.x / 960.0 - 1.0) << " " << (keypoints_3[i].pt.y / 960.0 - 1.0) << endl;

            good_match_id++;
        }
    }



    //We read each pair of images from the data directory, perfom keypoint,
    //detection, matching, and write it on the text file
    //string output = "../../output/bundle_data.txt";

    ofstream outfile(argv[4]);

    outfile << good_match_id << endl;
    outfile << out.str();

    outfile.close();

    //write2File(output,Frames,keypoints_1,keypoints_2,good_matches,num_cameras);

    /*Mat img_match;
    Mat img_goodmatch;
    drawMatches (img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
    drawMatches (img_1,keypoints_1,img_2,keypoints_2,good_matches,img_goodmatch);
    //drawMatches
    drawMatches (img_1,keypoints_1,img_2,keypoints_2,matches,img_match);

    imshow ("result low",img_match);
    imshow ("result high",img_goodmatch);
    waitKey(0);*/
    return 0;
}
