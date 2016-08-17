#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>
#include <fstream>

#include <opencv2/ccalib.hpp>

#include <vvs.h>
#include <grid_tracker.h>

using namespace std;
using namespace cv;
using namespace covis;

int main()
{
    vector<cv::Mat> im, im_all;
    cv::Mat im_k,im_tmp;
    vector<string> window, window_all;
    vector<vector<cv::Point> > cog, cog_all;
    vector<cv::Point> cog_k;
    vpHomogeneousMatrix Mreal1,Mtmp;
    vector<vpHomogeneousMatrix> Mreal;

    const unsigned int r = 8;
    const unsigned int c = 5;

    // load images and estimated pose
    string base = "../robot-images/";
    ifstream posefile((base+"robot_calib_2.txt").c_str());
    string line, imfile;
    if(posefile.is_open())
    {
        int count = 0;
        while(std::getline(posefile, line))
        {
            if(count == 0) {}
            else if(count == 1)
                ReadMatrix(line, Mreal1);
            else
            {
                // try to read image
                stringstream ss(line);
                ss >> imfile;
                cout << "Loading " << base+imfile << endl;
                im_tmp = cv::imread(base+imfile);

                if(cv::findChessboardCorners(im_tmp, Size(r,c), cog_k, cv::CALIB_CB_ADAPTIVE_THRESH))
                {
                    //  DrawSeq(imfile, im_tmp, cog_k);
                    //  cv::waitKey(0);

                    // save variables
                    im_all.resize(im_all.size()+1);
                    im_tmp.copyTo(im_all[im_all.size()-1]);
                    cog_all.push_back(cog_k);
                    window_all.push_back(imfile);

                    // read corresponding matrix
                    ReadMatrix(ss, Mtmp);
                    Mtmp = Mtmp*Mreal1.inverse();
                    Mreal.push_back(Mtmp);
                }
                else
                    cout << "no chessboard found" << endl;
            }
            if(count++ > 10)
                //break;
                1;

        }
    }

    // calibrate from random images
    unsigned int keep = 10;
    vector<unsigned int> idx;idx.clear();
    unsigned int k = rand()%im_all.size();
    cog.resize(keep);
    im.resize(keep);
    window.resize(keep);
    cout << "calibrating from: ";
    for(unsigned int i=0;i<keep;++i)
    {
        while(std::find(idx.begin(),idx.end(),k) != idx.end())
            k = rand()%im_all.size();
        cog[i] = cog_all[k];
        im_all[k].copyTo(im[i]);
        window[i] = window_all[k];
        //   DrawSeq(window[i], im[i], cog[i]);
        idx.push_back(k);
        cout << window[i] << " ";
        //   cv::waitKey(0);
    }
    cout << endl;


    // initiate calibrator
    VVS vvs(0.03, r,c);

    // minim...
    // camera model with default parameters
    const double pxy = 0.5*(im[0].rows+im[0].cols);
    //PerspectiveCamera cam(pxy, pxy, 0.5*im[0].cols, 0.5*im[0].rows);
    DistortionCamera cam(pxy, pxy, 0.5*im[0].cols, 0.5*im[0].rows, 0, 1);
    vvs.cam(cam);

    // display info during minimization
    vvs.SetDisplay(window, im);

    // calib from all images
    vvs.Calib(cog);

    // display results
    vvs.Display();

    // print results
    cout << "Intrinsic: " << cam.xi_.t() << endl;
    cout << "Max reprojection error: " << dmax << " pixels" << endl;

    waitKey(0);
}
