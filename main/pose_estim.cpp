#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>
#include <opencv2/ccalib.hpp>

#include <perspective_camera.h>
#include <distortion_camera.h>
#include <grid_tracker.h>
#include <vvs.h>

using namespace std;
using namespace cv;

const unsigned int CAM_MAIN = 0;
const unsigned int CAM_WARP = 1;
const bool USE_WARP = false;

int main()
{
    cv::VideoCapture cap(CAM_MAIN);

    CBTracker tracker(7,7);

    vector<cv::Mat> im(1);
    cv::Mat imtmp;
    vector<string> window(1);
    window[0] = "Camera";
    vector<vector<cv::Point> >cog(1);

    // load image for warping
    cv::VideoCapture cap2;
    cv::Mat im_ext;
    vector<cv::Point2f> warp1(4), warp0;

    if(USE_WARP)
    {
        cap2.open(CAM_WARP);
        cap2 >> im_ext;
        warp0.push_back(cv::Point(0,0));
        warp0.push_back(cv::Point(im_ext.cols,0));
        warp0.push_back(cv::Point(im_ext.cols,im_ext.rows));
        warp0.push_back(cv::Point(0,im_ext.rows));

    }
    vector<vpPoint> W(4);
    W[2].setWorldCoordinates(-0.145, -0.105, 0);
    W[3].setWorldCoordinates(0.145, -0.105, 0);
    W[0].setWorldCoordinates(0.145,0.105,  0);
    W[1].setWorldCoordinates(-0.145, 0.105, 0);

    // minim...
    // HP laptop cam + VVS
    DistortionCamera cam(646.7696914,  643.7703183 , 305.4480097, 242.7912928 , 0.00594527338 , 1.001051828, true);
    //PerspectiveCamera cam(650.9187166,  648.3355392,  309.6989075,  242.0794704, true);
    //VVS vvs(0.025, 8,6);
    VVS vvs(0.03, 7,7);
        vvs.cam(cam);


    // display
    vvs.SetDisplay(window, im, true);
    cv::namedWindow(window[0]);
    int clic_evt = cv::EVENT_FLAG_ALTKEY;

    bool first = true;
    unsigned int wait = 1000./cap.get(cv::CAP_PROP_FPS);


    unsigned int iter = 0;
    bool reset = false;
    vpHomogeneousMatrix M0;
    vpPoseVector pose;
    while(cap.isOpened())
    {
        cap.read(im[0]);
        im[0].copyTo(imtmp);
        cv::imshow(window[0], im[0]);

        if(tracker.Track(im[0]))
        {
            cog[0] = tracker.x_;

            vvs.Calib(cog, reset);
            pose.buildFrom(M0.inverse()*vvs.M_[0]);
            reset = vvs.display_ = (pose.getTranslationVector().euclideanNorm() > 0.1);
            M0 = vvs.M_[0];

            vvs.Display(wait);

            // trying warp
            if(USE_WARP)
            {
                cap2 >> im_ext;
                double u,v;
                cv::Mat im_warp, im_warp_g, mask, mask_inv, im_bg, warp_fg, im_all;
                for(unsigned int i=0;i<4;++i)
                {
                    W[i].track(vvs.M_[0]);
                    cam.Project(W[i], u,v);
                    warp1[i] = cv::Point2f(u,v);
                }
                cv::warpPerspective(im_ext, im_warp, cv::getPerspectiveTransform(warp0, warp1),cv::Size(im[0].cols, im[0].rows));
                //cv::imshow("Warp", im_warp);
                cv::cvtColor(im_warp, im_warp_g, cv::COLOR_BGR2GRAY);
                cv::threshold(im_warp_g, mask, 1, 255, cv::THRESH_BINARY);
                cv::bitwise_not(mask, mask_inv);
                cv::bitwise_and(im[0], im[0],im_bg, mask_inv);
                cv::bitwise_and(im_warp, im_warp,warp_fg, mask);
                cv::add(im_bg, im_warp, im_all);

                /*
            for(unsigned int i=0;i<im[0].rows;++i)
            {
                for(unsigned int j=0;j<im[0].cols;++j)
                {

                    cout << im_warp_g.at<unsigned int>(i,j) << endl;
                    if(im_warp_g.at<unsigned int>(i,j))
                        im[0].at<cv::Vec3b>(i,j) = im_warp.at<cv::Vec3b>(i,j);
                }
            }*/
                cv::imshow("Warp", im_all);
            }
        }

        cv::waitKey(wait);
    }
}




