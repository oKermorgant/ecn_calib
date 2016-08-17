#ifndef CAMERA_GENERIC_H
#define CAMERA_GENERIC_H


#include <visp/vpFeatureBuilder.h>
#include <visp/vpPoint.h>
#include <visp/vpMatrix.h>
#include <visp/vpSubMatrix.h>
#include <vector>
#include <opencv2/core/core.hpp>


namespace covis
{

class GenericCamera
{
public:

    vpColVector xi_, xi0_;

    GenericCamera() {}
    // get the number of intrinsic parameters
    unsigned int nbParam() {return xi_.getRows();}

    virtual void project(const vpPoint &_P, double &_u, double &_v) = 0;
    virtual void computeJacobianIntrinsic(const vpPoint &_P, vpMatrix &_J) = 0;
    virtual void computeJacobianExtrinsic(const vpPoint &_P, vpMatrix &_J) = 0;
    virtual void updateIntrinsic(const vpColVector &_dxi) = 0;
};



}



#endif // CAMERA_GENERIC_H
