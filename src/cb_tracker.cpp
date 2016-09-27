#include <cb_tracker.h>

CBTracker::CBTracker(unsigned int r, unsigned int c)
{
    size_ = Size(r, c);
}


bool CBTracker::Detect(cv::Mat &_im, vector<cv::Point> &_cog)
{
    cout << "Tracking..." << endl;
    bool success = cv::findChessboardCorners(_im, size_, _cog, cv::CALIB_CB_ADAPTIVE_THRESH);
    if(success)
        cog_prev_ = _cog;
    return success;

}

bool CBTracker::Track(cv::Mat &_im, vector<cv::Point> &_cog)
{
    cout << "Tracking..." << endl;
    bool success = cv::findChessboardCorners(_im, size_, _cog, cv::CALIB_CB_ADAPTIVE_THRESH);

    if(cog_prev_.size() && success)
    {
        Point x00 = cog_prev_[0], x0f = cog_prev_[cog_prev_.size()-1], x0 = _cog[0];
        if(sqdist(x00,x0) > sqdist(x0f, x0))
        {
            for(unsigned int i=0;i<cog_prev_.size();++i)
                cog_prev_[i] = _cog[cog_prev_.size()-i-1];
            _cog = cog_prev_;
        }
        else
            cog_prev_ = _cog;
    }


    return success;
}

