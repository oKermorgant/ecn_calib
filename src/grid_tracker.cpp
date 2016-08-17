#include <grid_tracker.h>
#include <visp/vpSubColVector.h>
#include <opencv2/core/core.hpp>
#include <visp/vpExponentialMap.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <visp/vpImageConvert.h>
#include <algorithm>
#include <grid_tracker.h>

using namespace std;
using namespace cv;
using namespace covis;

bool GridTracker::Detect(Mat &_im, vector<Point> &_cog)
{
    cv::Mat imfind;
    _im.copyTo(imfind);
    if(FindDots(imfind, _cog))
        // we know it did not work
        return false;
    // find correspondance in auto mode
    InitAuto(_cog);

    // init tracking
    vpImageConvert::convert(_im, I_);
    dots_.resize(_cog.size());
    vpImagePoint ip;
    for(unsigned int i=0;i<dots_.size();++i)
    {
        ip.set_uv(_cog[i].x, _cog[i].y);
        try
        {
            dots_[i].initTracking(I_, ip);
        }
        catch(...)
        {
            return false;
        }


    }
    cout << "Dots initialized" <<  endl;
    return true;
}



bool GridTracker::Track(Mat &_im, vector<cv::Point> &_cog)
{
    // instead of detecting from nothing, try to
    // convert image
    vpImageConvert::convert(_im, I_);

    // if dots initialized, just track
    for(unsigned int i=0;i<dots_.size();++i)
    {
        try
        {
            dots_[i].track(I_);
        }
        catch(...)
        {
            cout << "Lost dot#" << i << endl;
            return Detect(_im, _cog)            ;
        }
    }

    // write dots in cog
    vpImagePoint ip;
    for(unsigned int i=0;i<dots_.size();++i)
    {
        ip = dots_[i].getCog();
        _cog[i].x = ip.get_u();
        _cog[i].y = ip.get_v();
    }

    return true;
}



