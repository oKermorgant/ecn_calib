
#include <vvs.h>
#include <visp/vpSubColVector.h>
#include <opencv2/core/core.hpp>
#include <visp/vpExponentialMap.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace covis;
using std::cout;
using std::endl;
using std::vector;
using std::string;

/* Calibrate the camera from sequence of images and extracted points
    _pat is a list of image patterns with:
        - _pat[i].im is the i-eth image
        - _pat[i].point are the pixel points extracted from this image
        - _pat[i].window is the name of the window to display the results

*/
void VVS::calibrate(const std::vector<Pattern> &_pat)
{




}


/* Compute the pose of the camera from a single image and extracted points and a guess on the current pose M
    _pat is an image patterns with:
        - _pat[i].im is the i-eth image
        - _pat[i].point are the pixel points extracted from this image
        - _pat[i].window is the name of the window to display the results

*/
void VVS::computePose(const Pattern &_pat, vpHomogeneousMatrix &_M)
{




}
