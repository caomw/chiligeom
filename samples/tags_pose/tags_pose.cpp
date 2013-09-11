/*******************************************************************************
*   Copyright 2013 EPFL                                                        *
*                                                                              *
*   This file is part of chiligeom.                                            *
*******************************************************************************/

// This file serves as an illustration of how to use chiligeom::Reconstruction.
// It requires the chilitags library for 2D tag recognition.


// This header contains the detection part
#include <DetectChilitags.hpp>
// This header provides an easy way to use the tag detection information
#include <Chilitag.hpp>

// OpenCV goodness for I/O
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp> // for Point3_

#include <Reconstruction.hpp>

#include <iostream>
#include <vector>
#include <cmath> // isnan

using namespace std;
using namespace Reconstruction;

// OpenCV key codes
#define Q_KEY 1048689

#define NUM_ELTS 20 // max num of elements for stats
template < typename T >
class CircularVector
{
public:
    CircularVector() : idx(0)
    {
        data = vector<T>(NUM_ELTS);
    }
    void push_back(T& elt)
    {
        data[ idx++ % NUM_ELTS ] = elt;
    }

    vector<T> data;

private:
    int idx;
};


class SimpleStats
{
public:
    CircularVector<float> vals;

    void add(float x)
    {
        vals.push_back(x);
    }

    float mean()
    {
        float sum = 0.0;
        for(float a : vals.data) sum += a;
        return sum/NUM_ELTS;
    }

    float variance()
    {
        float current_mean = mean();
        float temp = 0;
        for(float a : vals.data)
            temp += (current_mean-a)*(current_mean-a);
        return temp/NUM_ELTS;
    }

    float stddev()
    {
        return sqrt(variance());
    }
};

int main(int argc, char* argv[])
{

    // Minimalistic filtering + computation of standard deviation
    SimpleStats filterX, filterY, filterZ;

    float tagRecoDuration, poseEstimationDuration;

    cout.precision(1); // format floats.
    cout.setf(ios::fixed, ios::floatfield);

    const static cv::Scalar scColor(255, 0, 255);
    // These constants will be given to OpenCv for drawing with
    // sub-pixel accuracy with fixed point precision coordinates
    static const int scShift = 16;
    static const float scPrecision = 1<<scShift;


    int tXRes = 640;
    int tYRes = 480;
    int tCameraIndex = 0;

    // The source of input images
    cv::VideoCapture tCapture(tCameraIndex);
    if (!tCapture.isOpened())
    {
        std::cerr << "Unable to initialise video capture." << std::endl;
        return 1;
    }
    tCapture.set(cv::CAP_PROP_FRAME_WIDTH, tXRes);
    tCapture.set(cv::CAP_PROP_FRAME_HEIGHT, tYRes);

    cv::namedWindow("TagPoseEstimation");

    int64 startTickCount;

    // The tag detection happens in the DetectChilitags class.
    // All it needs is a pointer to a OpenCv Image, i.e. a cv::Mat *
    // and a call to its update() method every time the image is updated.
    cv::Mat tInputImage;
    chilitags::DetectChilitags tDetectChilitags(&tInputImage);

    // Main loop, exiting when 'q is pressed'
    for (; Q_KEY != cv::waitKey(10); ) {

        // Capture a new image.
        tCapture.read(tInputImage);
        cv::flip(tInputImage, tInputImage, 1);

        // Detect tags on the current image.
        startTickCount = cv::getTickCount();
        tDetectChilitags.update();
        tagRecoDuration = (1000 * (double) (cv::getTickCount()-startTickCount)) / cv::getTickFrequency();

        // We dont want to draw directly on the input image, so we clone it
        cv::Mat tOutputImage = tInputImage.clone();

        // We iterate over the 1024 possible tags (from #0 to #1023)
        for (int tTagId = 0; tTagId < 1024; ++tTagId) {

            chilitags::Chilitag tTag(tTagId);
            if (tTag.isPresent()) {

                chilitags::Quad tCorners = tTag.getCorners();

                startTickCount = cv::getTickCount();

                // call the reconstruction algorithm.
                // Here, 30. is the width (and height) of our markers,
                // 800 (and 800) are the X (and Y) focal lengths of the
                // camera. These values can be computed with OpenCV's
                // calibration sample program, for instance.
                vector<cv::Point3f> pose = reconstruct(tCorners.points, 30.f, 30.f, 800, 800);
                poseEstimationDuration = (1000 * (double) (cv::getTickCount()-startTickCount)) / cv::getTickFrequency();

                // filter a bit the pose
                filterX.add(pose[0].x);
                filterY.add(pose[0].y);
                filterZ.add(pose[0].z);

                cout << "\x1b[KFirst corner of the first tag: " 
                    << filterX.mean() 
                    << "mm (σ: " << filterX.stddev() << "mm), " 
                    << filterY.mean() 
                    << "mm (σ: " << filterY.stddev() << "mm), " 
                    << filterZ.mean()
                    << "mm (σ: " << filterZ.stddev() << "mm). " 
                    << "Tag detec.: " << tagRecoDuration 
                    << "ms, pose estim.: " << poseEstimationDuration << "ms."
                    << "\x1b[1F" << endl; 
                
                // We draw this quadrilateral
                for (size_t i = 0; i < 4; ++i) {
                    cv::line(
                        tOutputImage,
                        scPrecision*tCorners[i],
                        scPrecision*tCorners[(i+1)%4],
                        scColor, 1, CV_AA, scShift);
                }

            break; // only track the first tag found.
            }
        }

        cv::imshow("TagPoseEstimation", tOutputImage);
    }

    cout << endl;
    cv::destroyWindow("TagPoseEstimation");
    tCapture.release();

    return 0;
}
