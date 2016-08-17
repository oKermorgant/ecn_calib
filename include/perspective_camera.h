#ifndef PERSPECTIVE_CAMERA_H
#define PERSPECTIVE_CAMERA_H

#include <generic_camera.h>

/*
 * This class inherits from GenericCamera and implements the classical perspective camera model
 *
 *
 *
 *
 */


namespace covis
{


// xi = (px, py, u0, v0)
class PerspectiveCamera : public GenericCamera
{
public:

    vpFeaturePoint p_;
    vpMatrix Lxy_;

    PerspectiveCamera(const double &_px, const double &_py, const double &_u0, const double &_v0, const bool &_calibrated=false)
    {
        xi0_.resize(4);
        xi0_[0] = _px;
        xi0_[1] = _py;
        xi0_[2] = _u0;
        xi0_[3] = _v0;
        Lxy_.resize(2,2);
        calibrated_ = _calibrated;
        Reset();
    }

    // reset to initial value
    void Reset()
    {
        xi_ = xi0_;
        Lxy_[0][0] = xi_[0];
        Lxy_[1][1] = xi_[1];
    }

    // compute pixel coordinates of a 3D point
    // we assume the point is already projected in the camera frame
    void Project(const vpPoint &_P, double &_u, double &_v)
    {
        _u = xi_[0]*_P.get_x() + xi_[2];    // u = px.x + u0
        _v = xi_[1]*_P.get_y() + xi_[3];    // v = py.y + v0
    }

    // write the sub-matrix of the Jacobian that corresponds to the given point and image
    void UpdateJacobian(const unsigned int &_row, const unsigned int &_image_idx, const vpPoint &_P, vpMatrix &_J)
    {
        // corresponding row

        // wrt intrinsic
        if(!calibrated_)
        {
            J_.init(_J, _row, 0, 2, 4);
            J_[0][0] = _P.get_x();
            J_[1][1] = _P.get_y();
        }

        // wrt extrinsic
        if(!calibrated_)
            J_.init(_J, _row, 4+6*_image_idx, 2, 6);
        else
            J_.init(_J, _row, 6*_image_idx, 2, 6);
        vpFeatureBuilder::create(p_, _P);
        J_ = Lxy_ * p_.interaction();
    }


    // check that the parameters stay meaningfull in case of wrong update
    void Update(const vpColVector &_dxi)
    {
        // update
        xi_ += _dxi;
        // all parameters should be positive
        for(unsigned int i=0;i<4;++i)
            if(xi_[i] < 0)
                xi_[i] = 0;

        // write Lxy
        Lxy_[0][0] = xi_[0];
        Lxy_[1][1] = xi_[1];
    }
};


}





#endif // PERSPECTIVE_CAMERA_H
