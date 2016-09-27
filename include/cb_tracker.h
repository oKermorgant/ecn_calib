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
    vector<Point> cog_prev_;

    CBTracker(unsigned int r, unsigned int c);

    bool Detect(cv::Mat &_im, vector<cv::Point> &_cog);
    bool Track(cv::Mat &_im, vector<cv::Point> &_cog);

private:
    inline double sqdist(cv::Point &p1, cv::Point &p2)
    {
        return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
    }
};

#endif // CBTRACKER_H
