#ifndef CBTRACKER_H
#define CBTRACKER_H

#include <opencv2/ccalib.hpp>
#include <visp/vpImageConvert.h>

using namespace cv;
using namespace std;

class CBTracker
{
public:

    Size size_;
    vector<Point> x_, x0_;

    CBTracker(unsigned int r, unsigned int c);

    bool Track(cv::Mat _im, bool _detect = false);
};

#endif // CBTRACKER_H
