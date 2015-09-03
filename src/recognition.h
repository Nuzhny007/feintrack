#pragma once

#include "feintrack_params.h"
#include "utils.h"
#include "some_types.h"
#include "feintrack_objects.h"

#if USE_HOG_RECOGNIZE
#if (CV_VERSION_MAJOR > 2)
#include <opencv2/core.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <opencv2/photo.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv2/contrib.hpp>
#include <opencv2/ml.hpp>

#include <opencv2/core/ocl.hpp>

#else // (CV_VERSION_EPOCH <= 2)

#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>

//#include <opencv2/photo/photo.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/ml/ml.hpp>

//#include <opencv2/ocl/ocl.hpp>

#endif
#endif
////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
	////////////////////////////////////////////////////////////////////////////

	// Распознавание объектов
	class CRecognition
	{
	public:
		CRecognition();
		~CRecognition();

		// Распознавание объекта
        object_types recognize_object(const CObjectRegion& region, const uchar* buf, uint32_t pitch, uint32_t frame_width, uint32_t frame_height, const mask_type* mask);

	private:
		// Является ли данная область похожей на голову
		bool is_head(int head_height, const hist_cont& h_hist, double &head_area, double& head_center, double& head_r_mu);
		// Является ли данная область похожей на тело
		bool is_body(int body_height, const hist_cont& h_hist, double head_center, double head_r_mu, double legs_center, double& body_area);
		
		// Распознавание людей
        object_types is_human(const CObjectRegion& region, const uchar* buf, uint32_t pitch, uint32_t frame_width, uint32_t frame_height, const mask_type* mask);
		// Распознавание автомобиля
        object_types is_vehicle(const CObjectRegion& region, uint32_t frame_width, const mask_type* mask);
		// Распознавание животного
        object_types is_animal(const CObjectRegion& region, uint32_t frame_width, const mask_type* mask);

#if USE_HOG_RECOGNIZE
		cv::HOGDescriptor hog; // Распознавание людей с помощью OpenCV'шного HOG
#endif
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
