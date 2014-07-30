#pragma once

#include "feintrack_params.h"
#include "utils.h"
#include "some_types.h"
#include "feintrack_objects.h"
////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	// ����������� �������� ��������� ����� � �������
	class CSegmentation
	{
	public:
		CSegmentation();
		~CSegmentation();

		void init(uint frame_width_, uint frame_height_, bool& use_cuda_); // ������������� ���������� ��������

#if ADV_OUT
		void draw_mask(bool use_cuda, uchar* adv_buf_rgb24);           // ����� �� �������������� ����� ����������� ��������� ����
#endif

		void morphology_open(bool use_cuda); // �������� �������������� ���������� "��������" ��� ����������� ��������� ����

		void square_segmentation(regions_container& regions);                   // ����������� �������� ��� ������������� ��������
		void iterative_segmentation(regions_container& regions);                // ����������� ����������� ��������� ����������� �� ������ 8-�� ���������
		void recursive_segmentation(regions_container& regions);                // ����������� ����������� ��������� ����������� �� ������ 8-�� ���������

#ifdef USE_CUDA
        void cuda_segmentation(regions_container& regions);                     // ����������� �������� � �������������� CUDA
		void copy_gpu2cpu();                          // ����������� ����� �� ����� � ��������� ������
#endif

		void set_show_objects(bool show_objects);     // ����������/�� ���������� �������

        mask_cont& get_mask();                        // ���������� ����� �����
#ifdef USE_CUDA
		CCudaBuf<mask_type, true>& get_device_mask(); // ��������� ����� �� �����������
#endif

	private:

		typedef std::vector<std::pair<mask_type, mask_type> > eq_regions_cont;
		eq_regions_cont eq_regions;              // ������ ��������������� �������� ��� ����������� �����������
		static bool pairs_comp(const eq_regions_cont::value_type &p1, const eq_regions_cont::value_type &p2); // C�������� ������������� ��������
		typedef std::vector<std::pair<mask_type, CObjectRegion> > tmp_regions_cont;
		tmp_regions_cont tmp_regions;            // ��������� ������ �������� ��� ����������� �����������
		void search_components(mask_type label, uint x, uint y, CObjectRegion& reg); // ����������� ����� ������� �������� ��� ����������� ��������

		mask_cont pixels_l;                           // ������ �������� frame_width * frame_height, � ������� �������� ��������� ��������� ����

#ifdef USE_CUDA
        CCudaBuf<mask_type, true> d_mask;             // ����������� ��� �����
		CCudaBuf<mask_type, true> d_mask_temp;        // ����������� ��� �������� �������������� ����������
		CCudaBuf<reg_label, true> d_reg;              // ����������� ��� ��������������� �����������

		CCudaBuf<reg_label, false> h_reg;             // ����������� ������ ��� ��������������� �����������
#endif

		mask_cont morphology_buf;                     // ������ �������� frame_width * frame_height ��� �������� �������������� ���������� �������� �������������� ���������� "��������"

		uint frame_width;                             // ������
		uint frame_height;                            // � ������ ������ ����� � ��������

		void add_to_region(regions_container& regions, int x, int y); // ����� ����������� �������
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
