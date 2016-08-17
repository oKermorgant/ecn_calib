#ifndef VVS_H
#define VVS_H

#include <visp/vpPoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <opencv2/core/core.hpp>
#include <visp/vpAdaptiveGain.h>

#include <generic_camera.h>

namespace covis
{

class VVS
{
public:
    VVS(const double &_d, const unsigned int r = 6, const unsigned int c = 6);
    void cam(GenericCamera &_cam);

    void SetDisplay(std::vector<std::string> &_window, std::vector<cv::Mat> &_im, const bool &_display_during_minim = false);
    void Display(const unsigned int &_wait = 0 ,const int &_only = -1);

    void Calib(std::vector<std::vector<cv::Point> > &_cog, const bool &_reset = false, const bool &_filter = false);

    inline double sgn(const double x, const double approx = 0)
    {
        if(fabs(x) < approx) return 0;
        return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }

    // 3D points + borders + frame
    unsigned int r_, c_;
    std::vector<vpPoint> X_, B_, F_;
    // Jacobian and features
    vpMatrix J_;
    vpColVector s_, sd_;
    std::vector<cv::Point2d> Bpx_;
    // used camera and markers
    GenericCamera* cam_;
    double d_;
    unsigned int n_xi_;
    // number of images that are used, good to know if nothing
    unsigned int nim_;
    // if we display
    bool display_;
    std::vector<std::string> *window_;
    std::vector<cv::Mat> *im_;
    cv::Mat imtp_;

    // optimization variables
    vpAdaptiveGain lambda_;
    std::vector<vpHomogeneousMatrix> M_;
    vpHomogeneousMatrix M0_;

    // history for plots
    vpMatrix xi_hist_, err_hist_;

};

}


#endif // VVS_H
