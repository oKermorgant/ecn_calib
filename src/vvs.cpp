
#include <vvs.h>
#include <visp/vpSubColVector.h>
#include <opencv2/core/core.hpp>
#include <visp/vpExponentialMap.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <visp/vpImageConvert.h>

// Initialize class with given grid size and dimension
VVS::VVS(const double &_d, const unsigned int r, const unsigned int c)
{
    r_ = r;
    c_ = c;
    X_.clear();
    for(unsigned int j=0;j<c;++j)
        for(unsigned int i=0;i<r;++i)
            X_.push_back(vpPoint((i-(r/2.-0.5))*_d,(j-(c/2.-0.5))*_d,0));

    // grid borders
    unsigned int nbcontour = (r+c+2)*2+1;
    B_.resize(nbcontour);
    cout << "Border length: " << nbcontour << endl;
    B_[0].setWorldCoordinates(-(r/2.+0.5)*_d, -(c/2.+0.5)*_d, 0);
    double dx, dy;
    for(unsigned int i=1;i<nbcontour-1;++i)
    {
        // update direction
        if(sgn(fabs(B_[i-1].get_oX())-(r/2.+0.5)*_d, _d/2) == 0 &&
           sgn(fabs(B_[i-1].get_oY())-(c/2.+0.5)*_d, _d/2) == 0)    // corner
        {
            dx = -_d*sgn(sgn(B_[i-1].get_oX())+sgn(B_[i-1].get_oY()),0.5);
            dy = _d*sgn(sgn(B_[i-1].get_oX())-sgn(B_[i-1].get_oY()),0.5);
        }
        B_[i].setWorldCoordinates(B_[i-1].get_oX()+dx, B_[i-1].get_oY()+dy, 0);
    }
    B_[nbcontour-1] = B_[0];
    Bpx_.resize(nbcontour);

    // frame
    F_.resize(4);
    F_[0].setWorldCoordinates(0, 0, 0);
    F_[1].setWorldCoordinates(-3*_d, 0, 0);
    F_[2].setWorldCoordinates(0, 3*_d, 0);
    F_[3].setWorldCoordinates(0, 0, -3*_d);

    lambda_ = vpAdaptiveGain(2, 0.1,0.1);
    M_.clear();
    display_ = false;
}

// Set the camera
void VVS::cam(GenericCamera &_cam)
{
    cam_ = &_cam;
    n_xi_ = _cam.calibrated_?0:cam_->NbParam();
}



void VVS::SetDisplay(vector<string> &_window, vector<cv::Mat> &_im, const bool &_display)
{
    display_ = _display;
    window_ = &_window;
    im_ = &_im;
}


// display projected grid vs real one, assumes points have been projected in s and sd
void VVS::Display(const unsigned int &_wait, const int &_only)
{
        unsigned int im_min = 0;
        unsigned int im_max = im_->size();
        if(_only != -1)
        {
            im_min = _only;
            im_max = _only+1;
        }

        unsigned int row;
        for(unsigned int n=im_min;n<im_max;++n)
        {
            im_->at(n).copyTo(imtp_);
            for(unsigned int i=0;i<X_.size();++i)
            {
                row = 2*i+2*r_*c_*n;
                cv::circle(imtp_, cv::Point(int(s_[row]), int(s_[row+1])), 1, cv::Scalar(0,0,255), 2);
                cv::circle(imtp_, cv::Point(int(sd_[row]), int(sd_[row+1])), 1, cv::Scalar(0,255,0), 2);
                cv::line(imtp_, cv::Point(int(s_[row]), int(s_[row+1])),
                        cv::Point(int(sd_[row]), int(sd_[row+1])), cv::Scalar(0,255,0), 1);
            }

            // projection of borders if display
            B_[0].track(M_[n]);
            cam_->Project(B_[0], Bpx_[0].x, Bpx_[0].y);
            for(unsigned int i=1;i<(r_+c_+2)*2+1;++i)
            {
                B_[i].track(M_[n]);
                cam_->Project(B_[i], Bpx_[i].x, Bpx_[i].y);
                cv::circle(imtp_, cv::Point(int(Bpx_[i].x), int(Bpx_[i].y)), 1, cv::Scalar(0,0,255), 2);
                cv::line(imtp_,
                         cv::Point(int(Bpx_[i].x), int(Bpx_[i].y)),
                         cv::Point(int(Bpx_[i-1].x), int(Bpx_[i-1].y)),
                        cv::Scalar(0,0,255), 2);
            }

            // frame
            for(unsigned int i=0;i<4;++i)
                F_[i].track(M_[n]);
            double u,v;
            cam_->Project(F_[0], u,v);
            cv::Point F0 = cv::Point(int(u), int(v));
            cam_->Project(F_[1], u,v); cv::line(imtp_,F0,cv::Point(int(u),int(v)),cv::Scalar(0,0,255),2);              // X
            cam_->Project(F_[2], u,v); cv::line(imtp_,F0,cv::Point(int(u),int(v)),cv::Scalar(0,255,0),2);            // Y
            cam_->Project(F_[3], u,v); cv::line(imtp_,F0,cv::Point(int(u),int(v)),cv::Scalar(255,0,0),2);            // Z

            // display
            cv::imshow(window_->at(n), imtp_);
        }
        cv::waitKey(_wait);
}


void VVS::Calib(vector<vector<cv::Point> > &_cog, const bool &_reset, const bool &_filter)
{
    const unsigned int n_im = _cog.size();
    if(M_.size() == 0 || _reset)  // variables not initialized
    {
        s_.resize(2*r_*c_*n_im);
        sd_.resize(2*r_*c_*n_im);

        J_.resize(2*r_*c_*n_im, n_xi_+n_im*6);
        for(unsigned int i=0;i<r_*c_*n_im;++i)
            J_[2*i][2] = J_[2*i+1][3] = 1;

        M_.resize(n_im);
        // wild guess for initial pose, then will use the last found at initial value
        for(unsigned int n=0;n<n_im;++n)
            M_[n].buildFrom(0,0.,0.5,0,0,atan2(_cog[n][5].y-_cog[n][0].y, _cog[n][5].x-_cog[n][0].x));
        M0_[0][0] = 2;
    }

    // write cogs as desired features
    unsigned int row = 0;
    for(unsigned int n = 0;n<n_im;++n)
    {
        for(unsigned int i=0;i<r_*c_;++i)
        {
            row = 2*i+2*r_*c_*n;
            sd_[row] = _cog[n][i].x;
            sd_[row+1] = _cog[n][i].y;
        }
    }


    // loop variables
    vpColVector dxiv(n_xi_+n_im*6);
    vpSubColVector dxi(dxiv, 0, n_xi_);

    err_hist_.clear();
    xi_hist_.clear();


    unsigned int i, iter=0;
    bool carry_on;

    dxiv = 1;
    while(dxiv.euclideanNorm() > 0.00001 && iter++ < 100)
    {
        carry_on = true;

        // current projection and corresponding interaction matrix
        for(unsigned int n=0;n<n_im;++n)
        {
            for(i=0;i<r_*c_;++i)
            {
                X_[i].track(M_[n]);
                if(X_[i].get_Z() < 0)
                {
                    carry_on = false;
                    break;
                }

                row = 2*i+2*r_*c_*n;

                // corresponding pixels
                cam_->Project(X_[i], s_[row], s_[row+1]);

                // update Jacobian
                cam_->UpdateJacobian(row, n, X_[i], J_);
            }

            if(carry_on && display_)
                Display(1, n);
        }

        if(carry_on)
        {
            // minim
            dxiv = -lambda_((s_-sd_).euclideanNorm())*J_.pseudoInverse() * (s_ - sd_);
            //  cout << "current gain: " << lambda((s-sd).euclideanNorm()) << endl;

            // update and check intrinsic parameters
            if(!cam_->calibrated_)
                cam_->Update(dxi);

            // update extrinsic parameters for each image
            for(unsigned int n=0;n<n_im;++n)
            {
                vpSubColVector v(dxiv, n_xi_+6*n, 6);
                M_[n] = vpExponentialMap::direct(v).inverse() * M_[n];
            }
        }
        else
        {
            dxiv = 0;
            cout << "canceled" << endl;
        }




        // save iteration variables
        xi_hist_.stack(cam_->xi_.t());
        err_hist_.stack((s_-sd_).t());
    }


    // filter in SE3 for first matrix
    if(_filter)
    {
        if(M0_[0][0] != 2)
        {
            cout << "Was " << M0_.getTranslationVector().t() << endl;
        cout << "Found " << M_[0].getTranslationVector().t() << endl;
         /*   vpHomogeneousMatrix Md = ;
            cout << "Diff " << Md.getTranslationVector().t() << endl;

        vpColVector v = vpExponentialMap::inverse(Md);*/
        M_[0] = M0_*vpExponentialMap::direct(-0.*vpExponentialMap::inverse(M_[0].inverse()*M0_));
        cout << "Final " << M_[0].getTranslationVector().t() << endl << endl;
        }
        M0_ = M_[0];
    }
}
