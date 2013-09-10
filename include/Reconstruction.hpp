#ifndef Reconstruction_HPP
#define Reconstruction_HPP

#include <opencv2/opencv.hpp>
#include <vector>


namespace Reconstruction {


	template <class T> std::vector< cv::Point3_<T> > reconstruct(const std::vector< cv::Point_<T> > & pObservedQuadix, T pQuadixWidthMM, T pQuadixHeightMM, float pFocalX, float pFocalY);

}

#endif //Reconstructions_HPP
