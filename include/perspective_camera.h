#ifndef PERSPECTIVE_CAMERA_H
#define PERSPECTIVE_CAMERA_H

#include <generic_camera.h>

/*
 * This class inherits from GenericCamera and implements the classical perspective camera model
 * Methods to be filled up are:
 * - project
 * - computeJacobianIntrinsic
 * - computeJacobianExtrinsic
 */


namespace covis
{


// xi = (px, py, u0, v0)
class PerspectiveCamera : public GenericCamera
{
public:

    PerspectiveCamera(const double &_px, const double &_py, const double &_u0, const double &_v0, const bool &_calibrated=false)
    {
        xi_.resize(4);
        xi_[0] = _px;
        xi_[1] = _py;
        xi_[2] = _u0;
        xi_[3] = _v0;
    }


    // check that the parameters stay meaningfull in case of wrong update
    void updateIntrinsic(const vpColVector &_dxi)
    {
        // update
        xi_ += _dxi;
        // all parameters should be positive
        for(unsigned int i=0;i<4;++i)
            if(xi_[i] < 0)
                xi_[i] = 0;

    }


    // compute pixel coordinates of a 3D point
    // we assume the point is already projected in the camera frame
    void project(const vpPoint &_P, double &_u, double &_v)
    {

    }


    // write the Jacobian corresponding to the intrinsic parameters
    // Jacobian should be 2x4
    // we assume the point is already projected in the camera frame
    void computeJacobianIntrinsic(const vpPoint &_P, vpMatrix &_J)
    {




    }


    // write the Jacobian wrt extrinsic parameters
    // J should be 2x6
    // we assume the point is already projected in the camera frame
    void computeJacobianExtrinsic(const vpPoint &_P, vpMatrix &_J)
    {

    }
};
}


#endif // PERSPECTIVE_CAMERA_H
