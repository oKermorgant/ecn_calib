#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>
#include <fstream>

#include <opencv2/ccalib.hpp>

#include <vvs.h>
#include <grid_tracker.h>
#include <perspective_camera.h>
#include <distortion_camera.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;

using cv::waitKey;
using namespace covis;

int main()
{
    // load calibration images from hard drive
    const string base = "../images/";
    const string prefix = "img";

    // init empty vector of detected patterns
    vector<Pattern> patterns;
    patterns.clear();
    patterns.reserve(6);

    // this tracker detects a 6x6 grid of points
    GridTracker tracker;

    // read images while the corresponding file exists
    // images are displayed to ensure the detection was performed
    while(true)
    {
        stringstream ss;
        ss << prefix << patterns.size() << ".jpg";
        std::ifstream testfile(base + ss.str());
        if(testfile.good())
        {
            testfile.close();
            Pattern pat;
            pat.im =  cv::imread(base + ss.str());
            tracker.detect(pat.im, pat.point, false);
            pat.window = ss.str();
            // draw extraction results
            drawSeq(pat.window, pat.im, pat.point);
            patterns.push_back(pat);
            waitKey(0);
        }
        else
            break;
    }
    cout << "Found " << patterns.size() << " images" << endl;

    // create a camera model (Perspective or Distortion)
    // default parameters should be guessed from image dimensions
    PerspectiveCamera cam(1,1,1,1);

    // initiate virtual visual servoing with inter-point distance and pattern dimensions
    VVS vvs(cam, 0.03, 6, 6);

    // calibrate from all images
    vvs.calibrate(patterns);

    // print results


    // this will wait for a key pressed to stop the program
    waitKey(0);
}
