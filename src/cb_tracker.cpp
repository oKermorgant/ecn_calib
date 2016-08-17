#include <cb_tracker.h>

CBTracker::CBTracker(unsigned int r, unsigned int c)
{
    size_ = Size(r, c);
  //  x_.resize(r*c);
}


bool CBTracker::Track(cv::Mat _im, bool _detect)
{
   bool success = cv::findChessboardCorners(_im, size_, x_, cv::CALIB_CB_ADAPTIVE_THRESH);
   if(x0_.size() && !_detect)  // compare to previous to keep orientation
    {
        Point x00 = x0_[0], x0f = x0_[x0_.size()-1], x0 = x_[0];
        if((x00.x-x0.x)*(x00.x-x0.x)+(x00.y-x0.y)*(x00.y-x0.y) >
                (x0f.x-x0.x)*(x0f.x-x0.x)+(x0f.y-x0.y)*(x0f.y-x0.y))
        {
            for(unsigned int i=0;i<x0_.size();++i)
                x0_[i] = x_[x0_.size()-i-1];
            x_ = x0_;
        }
    }

    x0_ = x_;

    return success;
}
