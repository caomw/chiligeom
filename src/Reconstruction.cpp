#include "Reconstruction.hpp"

namespace Reconstruction {

	template <class T> std::vector< cv::Point3_<T> >  reconstruct(const std::vector< cv::Point_<T> > & pObservedQuadix, T pQuadixWidthMM, T pQuadixHeightMM, float pFocalX, float pFocalY){

		float tCameraFocalX = pFocalX;
		float tCameraFocalY = pFocalY;

		T tXS1=pObservedQuadix[0].x;
		T tYS1=pObservedQuadix[0].y;
		T tXS2=pObservedQuadix[1].x;
		T tYS2=pObservedQuadix[1].y;
		T tXS3=pObservedQuadix[2].x;
		T tYS3=pObservedQuadix[2].y;
		T tXS4=pObservedQuadix[3].x;
		T tYS4=pObservedQuadix[3].y;

		T tXS1_2 = pObservedQuadix[0].x*pObservedQuadix[0].x;
		T tXS2_2 = pObservedQuadix[1].x*pObservedQuadix[1].x;
		T tXS3_2 = pObservedQuadix[2].x*pObservedQuadix[2].x;
		T tXS4_2 = pObservedQuadix[3].x*pObservedQuadix[3].x;
		T tYS1_2 = pObservedQuadix[0].y*pObservedQuadix[0].y;
		T tYS2_2 = pObservedQuadix[1].y*pObservedQuadix[1].y;
		T tYS3_2 = pObservedQuadix[2].y*pObservedQuadix[2].y;
		T tYS4_2 = pObservedQuadix[3].y*pObservedQuadix[3].y;
		T tFx_2 = tCameraFocalX*tCameraFocalX;
		T tFy_2 = tCameraFocalY*tCameraFocalY;
		T tDiag_2 = pQuadixWidthMM*pQuadixWidthMM+pQuadixHeightMM*pQuadixHeightMM;
		T tFx = tCameraFocalX;
		T tFy = tCameraFocalY;

		T tZ4Temp1 = 
			4.0f*tXS1_2*tFy_2*tXS4_2*tYS3_2+
			tFx_2*tFy_2*tXS2_2*tYS4_2+
			4.0f*tYS1_2*tFx_2*tXS3_2*tYS4_2+
			tYS1_2*tFx_2*tXS3_2*tYS2_2+
			4.0f*tXS1_2*tFy_2*tXS4_2*tYS2_2+
			tXS1_2*tFy_2*tXS2_2*tYS3_2+
			tFx_2*tFy_2*tXS4_2*tYS3_2+
			tXS4_2*tFy_2*tXS2_2*tYS3_2+
			tFx_2*tFy_2*tXS4_2*tYS2_2+
			tXS4_2*tFy_2*tXS2_2*tYS1_2+
			tXS4_2*tFy_2*tXS3_2*tYS2_2+
			tXS4_2*tFy_2*tXS3_2*tYS1_2+
			tYS1_2*tFx_2*tXS4_2*tYS3_2+
			tYS1_2*tFx_2*tXS4_2*tYS2_2+
			tYS4_2*tFx_2*tXS3_2*tYS2_2-2*tYS4_2*tFx_2*tXS1*tYS3_2*tXS2-4*tFx_2*tFy_2*tXS1*tYS2_2*tXS3-4*tFx_2*tFy_2*tXS1*tYS3_2*tXS2+
			4.0f*tFx_2*tFy_2*tXS1*tYS3*tXS3*tYS2+
			4.0f*tFx_2*tFy_2*tXS2*tYS1*tXS3*tYS2-4*tFx_2*tFy_2*tXS2_2*tYS1*tYS3-4*tFx_2*tFy_2*tXS3_2*tYS1*tYS2+
			4.0f*tFx_2*tFy_2*tXS2_2*tYS3_2+
			4.0f*tFx_2*tFy_2*tXS3_2*tYS2_2-8*tFx_2*tFy_2*tXS2*tYS3*tXS3*tYS2+
			2.0f*tFx_2*tFy_2*tXS3*tYS1*tXS1*tYS2+
			tFx_2*tFy_2*tXS2_2*tYS1_2+
			4.0f*tFx_2*tFy_2*tXS3*tYS1*tXS2*tYS3+
			4.0f*tFx_2*tFy_2*tXS1*tYS2*tXS2*tYS3-2*tFx_2*tFy_2*tXS2*tYS1*tXS1*tYS2-2*tFx_2*tFy_2*tXS3*tYS1*tXS1*tYS3+
			tFx_2*tFy_2*tXS1_2*tYS3_2+
			tFx_2*tFy_2*tXS3_2*tYS1_2-2*tFx_2*tFy_2*tXS3*tYS1_2*tXS2+
			2.0f*tFx_2*tFy_2*tXS2*tYS1*tXS1*tYS3-2*tFx_2*tFy_2*tXS1_2*tYS3*tYS2+
			tFx_2*tFy_2*tXS1_2*tYS2_2-2*tYS4_2*tFx_2*tXS3*tYS2_2*tXS1+
			2.0f*tFx_2*tFy_2*tXS2*tYS4*tXS4*tYS3+
			tYS4_2*tFx_2*tXS1_2*tYS2_2-2*tYS4_2*tFx_2*tXS1_2*tYS3*tYS2-4*tYS1_2*tFx_2*tXS2_2*tYS3*tYS4-2*tFx_2*tFy_2*tXS3*tYS4*tXS4*tYS3-4*tFx_2*tFy_2*tXS3_2*tYS4*tYS2+
			4.0f*tYS1_2*tFx_2*tXS2*tYS4*tXS3*tYS2-2*tFx_2*tFy_2*tXS3*tYS4_2*tXS2-8*tYS1_2*tFx_2*tXS2*tYS4_2*tXS3-2*tYS1_2*tFx_2*tXS4_2*tYS3*tYS2+
			4.0f*tYS1_2*tFx_2*tXS2*tYS4*tXS4*tYS3-2*tYS1_2*tFx_2*tXS4*tYS2_2*tXS3-4*tFx_2*tFy_2*tXS4*tYS2_2*tXS3+
			2.0f*tFx_2*tFy_2*tXS1*tYS2_2*tXS4+
			tYS4_2*tFx_2*tXS2_2*tYS3_2+
			2.0f*tFx_2*tFy_2*tXS1*tYS2*tXS3*tYS4+
			2.0f*tFx_2*tFy_2*tXS3*tYS1*tXS4*tYS2-2*tFx_2*tFy_2*tXS4_2*tYS3*tYS2-4*tFx_2*tFy_2*tXS4*tYS3_2*tXS2-4*tXS1*tXS4_2*tFy_2*tXS2*tYS3_2-4*tYS1*tYS4_2*tFx_2*tXS2_2*tYS3+
			2.0f*tYS1*tYS4*tFx_2*tXS2_2*tYS3_2+
			2.0f*tYS1*tYS4*tFx_2*tXS3_2*tYS2_2+
			4.0f*tFx_2*tFy_2*tXS3*tYS4*tXS2*tYS3-4*tYS1*tYS4_2*tFx_2*tXS3_2*tYS2+
			2.0f*tYS1_2*tFx_2*tXS4*tYS3*tXS3*tYS2+
			4.0f*tYS1_2*tFx_2*tXS4*tYS2*tXS3*tYS4+
			2.0f*tFx_2*tFy_2*tXS3_2*tYS1*tYS4+
			4.0f*tFx_2*tFy_2*tXS4*tYS3*tXS3*tYS2+
			2.0f*tFx_2*tFy_2*tXS2_2*tYS1*tYS4-8*tXS1_2*tFy_2*tXS4_2*tYS3*tYS2+
			4.0f*tXS1_2*tFy_2*tXS4*tYS2*tXS2*tYS3-4*tXS1_2*tFy_2*tXS4*tYS3_2*tXS2+
			4.0f*tFx_2*tFy_2*tXS2*tYS4*tXS3*tYS2-2*tXS1_2*tFy_2*tXS2_2*tYS4*tYS3-2*tXS1_2*tFy_2*tXS2*tYS4_2*tXS3-2*tXS1_2*tFy_2*tXS3_2*tYS4*tYS2-4*tYS1_2*tFx_2*tXS3_2*tYS4*tYS2-2*tYS1_2*tFx_2*tXS2*tYS3_2*tXS4-4*tXS1_2*tFy_2*tXS3*tYS2_2*tXS4-4*tXS1*tXS4_2*tFy_2*tXS3*tYS2_2-2*tYS1_2*tFx_2*tXS2*tYS3*tXS3*tYS2+
			2.0f*tXS1*tXS4*tFy_2*tXS2_2*tYS3_2+
			2.0f*tYS1_2*tFx_2*tXS2*tYS3*tXS4*tYS2+
			2.0f*tXS1*tXS4*tFy_2*tXS3_2*tYS2_2-2*tXS1_2*tFy_2*tXS3*tYS2*tXS2*tYS3-4*tFx_2*tFy_2*tXS2_2*tYS4*tYS3+
			2.0f*tXS1*tXS4*tFy_2*tXS2_2*tYS1*tYS4-2*tXS1*tXS4*tFy_2*tXS2_2*tYS3*tYS4+
			4.0f*tXS1*tXS4_2*tFy_2*tXS2*tYS1*tYS3+
			4.0f*tYS1_2*tFx_2*tXS2*tYS3*tXS3*tYS4-4*tXS1*tXS4_2*tFy_2*tXS3*tYS1*tYS3+
			2.0f*tXS1*tXS4*tFy_2*tXS3_2*tYS1*tYS4-2*tXS1*tXS4*tFy_2*tXS3_2*tYS1*tYS2-2*tXS4_2*tFy_2*tXS2_2*tYS1*tYS3-2*tXS4_2*tFy_2*tXS3_2*tYS1*tYS2+
			tYS4_2*tFx_2*tXS1_2*tYS3_2-2*tXS4_2*tFy_2*tXS3*tYS1_2*tXS2-2*tFx_2*tFy_2*tXS2*tYS4*tXS4*tYS2+
			2.0f*tFx_2*tFy_2*tXS3*tYS4*tXS4*tYS2+
			2.0f*tFx_2*tFy_2*tXS1*tYS3_2*tXS4-2*tYS1*tYS4*tFx_2*tXS3*tYS2_2*tXS4+
			2.0f*tYS1*tYS4*tFx_2*tXS1*tYS2_2*tXS4-4*tXS1*tXS4_2*tFy_2*tXS2*tYS1*tYS2+
			4.0f*tYS1*tYS4_2*tFx_2*tXS1*tYS2*tXS3+
			tXS1_2*tFy_2*tXS3_2*tYS4_2-2*tYS1*tYS4*tFx_2*tXS1*tYS2_2*tXS3+
			tXS1_2*tFy_2*tXS2_2*tYS4_2-2*tYS1*tYS4*tFx_2*tXS1*tYS3_2*tXS2-4*tYS1*tYS4_2*tFx_2*tXS1*tYS2*tXS2-2*tYS1*tYS4*tFx_2*tXS2*tYS3_2*tXS4+
			4.0f*tXS1*tXS4_2*tFy_2*tXS3*tYS1*tYS2-2*tXS1*tXS4*tFy_2*tXS3_2*tYS2*tYS4+
			4.0f*tYS1*tYS4_2*tFx_2*tXS2*tYS3*tXS3+
			4.0f*tXS1*tXS4_2*tFy_2*tXS2*tYS3*tYS2+
			4.0f*tFx_2*tFy_2*tXS4*tYS2*tXS2*tYS3-2*tXS1*tXS4*tFy_2*tXS2_2*tYS1*tYS3+
			4.0f*tXS1*tXS4_2*tFy_2*tXS3*tYS2*tYS3+
			4.0f*tXS1_2*tFy_2*tXS2*tYS4*tXS4*tYS3+
			2.0f*tYS1*tYS4*tFx_2*tXS1*tYS3_2*tXS4+
			2.0f*tXS4_2*tFy_2*tXS3*tYS1*tXS2*tYS3-4*tYS1*tYS4_2*tFx_2*tXS1*tYS3*tXS3+
			4.0f*tYS1*tYS4_2*tFx_2*tXS1*tYS3*tXS2+
			tFx_2*tFy_2*tXS3_2*tYS4_2+
			4.0f*tYS1*tYS4_2*tFx_2*tXS3*tYS2*tXS2-2*tFx_2*tFy_2*tXS1*tYS3*tXS3*tYS4-4*tXS1_2*tFy_2*tXS4*tYS3*tXS3*tYS4+
			4.0f*tXS1_2*tFy_2*tXS4*tYS3*tXS3*tYS2-4*tFx_2*tFy_2*tXS3*tYS1*tXS2*tYS4-2*tFx_2*tFy_2*tXS3*tYS1*tXS4*tYS3-2*tFx_2*tFy_2*tXS1*tYS2*tXS2*tYS4+
			2.0f*tXS1_2*tFy_2*tXS3*tYS4*tXS2*tYS3+
			2.0f*tXS1_2*tFy_2*tXS2*tYS4*tXS3*tYS2-4*tYS1_2*tFx_2*tXS2*tYS4*tXS4*tYS2-4*tYS1_2*tFx_2*tXS4*tYS3*tXS3*tYS4+
			4.0f*tXS1_2*tFy_2*tXS3*tYS4*tXS4*tYS2+
			4.0f*tYS1_2*tFx_2*tXS2_2*tYS4_2+
			tXS1_2*tFy_2*tXS3_2*tYS2_2+
			tYS1_2*tFx_2*tXS2_2*tYS3_2-2*tXS4_2*tFy_2*tXS3*tYS2*tXS2*tYS3+
			2.0f*tXS4_2*tFy_2*tXS3*tYS2*tXS2*tYS1-4*tXS1_2*tFy_2*tXS2*tYS4*tXS4*tYS2+
			2.0f*tXS1*tXS4*tFy_2*tXS2*tYS1*tXS3*tYS2+
			2.0f*tYS1*tYS4*tFx_2*tXS1*tYS3*tXS3*tYS2+
			2.0f*tYS1*tYS4*tFx_2*tXS2*tYS3*tXS4*tYS2+
			2.0f*tYS1*tYS4*tFx_2*tXS3*tYS2*tXS4*tYS3+
			2.0f*tYS1*tYS4*tFx_2*tXS1*tYS2*tXS2*tYS3-4*tYS1*tYS4*tFx_2*tXS1*tYS3*tXS4*tYS2+
			2.0f*tFx_2*tFy_2*tXS1*tYS3*tXS2*tYS4-4*tFx_2*tFy_2*tXS1*tYS3*tXS4*tYS2+
			2.0f*tFx_2*tFy_2*tXS2*tYS1*tXS4*tYS3-2*tFx_2*tFy_2*tXS2*tYS1*tXS4*tYS2+
			2.0f*tYS4_2*tFx_2*tXS1*tYS2*tXS2*tYS3+
			2.0f*tYS4_2*tFx_2*tXS3*tYS2*tXS1*tYS3-2*tYS4_2*tFx_2*tXS3*tYS2*tXS2*tYS3+
			2.0f*tXS1*tXS4*tFy_2*tXS3*tYS1*tXS2*tYS3-4*tYS1*tYS4*tFx_2*tXS2*tYS3*tXS3*tYS2-4*tXS1*tXS4*tFy_2*tXS3*tYS1*tXS2*tYS4-4*tXS1*tXS4*tFy_2*tXS3*tYS2*tXS2*tYS3+
			2.0f*tXS1*tXS4*tFy_2*tXS3*tYS2*tXS2*tYS4+
			2.0f*tXS1*tXS4*tFy_2*tXS2*tYS3*tXS3*tYS4;

		T tZ4 = (( -1.0f/(tZ4Temp1)*sqrt((tZ4Temp1)*(tDiag_2))*(-tXS3*tYS1+tXS3*tYS2+tXS2*tYS1+tXS1*tYS3-tXS1*tYS2-tXS2*tYS3)*tFx*tFy));
		T tZ1 = tZ4*(tXS3*tYS4-tXS4*tYS3-tXS2*tYS4+tXS4*tYS2+tXS2*tYS3-tXS3*tYS2)/(-tXS2*tYS3+tXS2*tYS1+tXS1*tYS3+tXS3*tYS2-tXS3*tYS1-tXS1*tYS2);
		T tZ2 = (tXS3*tYS4-tXS4*tYS3+tYS1*tXS4-tYS4*tXS1-tXS3*tYS1+tXS1*tYS3)*tZ4/(-tXS2*tYS3+tXS2*tYS1+tXS1*tYS3+tXS3*tYS2-tXS3*tYS1-tXS1*tYS2);
		T tZ3 = -tZ4*(tXS2*tYS4-tXS2*tYS1-tYS4*tXS1-tXS4*tYS2+tYS1*tXS4+tXS1*tYS2)/(-tXS2*tYS3+tXS2*tYS1+tXS1*tYS3+tXS3*tYS2-tXS3*tYS1-tXS1*tYS2);

		T tX1 = tZ1 * tXS1 / tFx;
		T tX2 = tZ2 * tXS2 / tFx;
		T tX3 = tZ3 * tXS3 / tFx;
		T tX4 = tZ4 * tXS4 / tFx;
		T tY1 = tZ1 * tYS1 / tFy;
		T tY2 = tZ2 * tYS2 / tFy;
		T tY3 = tZ3 * tYS3 / tFy;
		T tY4 = tZ4 * tYS4 / tFy;

		//
		std::vector< cv::Point3_<T> > tRetQuadix;

		for (std::size_t i = 0;i<4;++i){

			tRetQuadix.push_back(cv::Point3_<T> ());

		}

		tRetQuadix[0].x=tX1;
		tRetQuadix[0].y=tY1;
		tRetQuadix[0].z=tZ1;
	
		tRetQuadix[1].x=tX2;
		tRetQuadix[1].y=tY2;
		tRetQuadix[1].z=tZ2;

		tRetQuadix[3].x=tX3;
		tRetQuadix[3].y=tY3;
		tRetQuadix[3].z=tZ3;

		tRetQuadix[2].x=tX4;
		tRetQuadix[2].y=tY4;
		tRetQuadix[2].z=tZ4;

		return tRetQuadix;

	}

	template std::vector< cv::Point3_<float> >  reconstruct(const std::vector< cv::Point_<float> > & pObservedQuadix, float pQuadixWidthMM, float pQuadixHeightMM, float pFocalX, float pFocalY);
	template std::vector< cv::Point3_<double> >  reconstruct(const std::vector< cv::Point_<double> > & pObservedQuadix, double pQuadixWidthMM, double pQuadixHeightMM, float pFocalX, float pFocalY);

}


