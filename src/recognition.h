#pragma once

#include "feintrack_params.h"
#include "utils.h"
#include "some_types.h"
#include "feintrack_objects.h"

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
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
