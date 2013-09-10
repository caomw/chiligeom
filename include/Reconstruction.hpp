/**
 * @file
 * @author  Andrea Mazzei <johnnyx811@gmail.com>
 * @version 1.0
 */

#ifndef Reconstruction_HPP
#define Reconstruction_HPP

#include <opencv2/opencv.hpp>
#include <vector>


namespace Reconstruction {

	/**
	
	It reconstruct the 3D coordinates of the 4 vertex of a quadrilateral from 2D observations and from the camera focal length

	@param	pObservedQuadix Quadrilater containing the 2D vertex pixel coordinates in a clockwise orientation
	@param	pQuadixWidthMM Width of the quadrilater in millimeters
	@param	pQuadixWidthMM Height of the quadrilater in millimeters
	@param	pFocalX Horizontal focal length of the camera
	@param	pFocalY Vertical focal length of the camera
	@return	Vector of the four 3D vertexes of the quadrilater in real world coordinates 
	*/
	template <class T> std::vector< cv::Point3_<T> > reconstruct(const std::vector< cv::Point_<T> > & pObservedQuadix, T pQuadixWidthMM, T pQuadixHeightMM, float pFocalX, float pFocalY);

}

#endif //Reconstructions_HPP
