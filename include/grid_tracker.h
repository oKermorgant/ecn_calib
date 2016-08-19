#ifndef GRIDTRACKER_H
#define GRIDTRACKER_H

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDot2.h>
#include <grid_detection.h>

namespace covis
{

class GridTracker
{
public:
    GridTracker() {dots_.clear();visible_.clear();}

    bool detect(cv::Mat &_im, std::vector<cv::Point> &_cog, const bool &_init_tracking = true);
    bool track(cv::Mat &_im, std::vector<cv::Point> &_cog);

    // dot tracking
    vpImage<unsigned char> I_;
    std::vector<vpDot2> dots_;
    std::vector<bool> visible_;
};

}


#endif // GridTracker_H
