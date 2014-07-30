#pragma once
#include <map>
#include <list>
#include <limits>

#include "feintrack_params.h"
#include "feintrack_objects.h"
#include "bgrnd_substr.h"
#include "segmentation.h"
#include "recognition.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////

	typedef std::basic_string<char> mstring;
	////////////////////////////////////////////////////////////////////////////

	// ��������� � ������������� ��������
	class CFeinTrack
	{
	private:
		static const float_t density_threshold;  // ����� ��������� �������. ������� � ������� ��������� ���� ����� ������ ����� ���������

		bool use_square_segmentation;            // ������������ ����������� �������� ��� ������������� ��������. ����� ������������ 8-�� ���������

		int left_object_time0;                   // �����, ����� �������� ������ ��������� � ������ ��������, �������� ���������� ������������
		int left_object_time1;                   // �����, ����� �������� ������ ��������� ������� �� �����������
		int left_object_time2;                   // �����, ����� �������� ������ ��������� �����������
		int left_object_time3;                   // �����, ����� �������� ����������� ������� ���������

		int left_object_time1_sec;               // ����� � ��������, ����� �������� ������ ��������� ������� �� �����������
		int left_object_time2_sec;               // ����� � ��������, ����� �������� ������ ��������� �����������
		int left_object_time3_sec;               // ����� � ��������, ����� �������� ����������� ������� ���������

        uint32_t frame_width;                    // ������
        uint32_t frame_height;                   // � ������ ������ ����� � ��������
		color_type curr_color_type;              // ������� ��� ��������� ������������ �������������� �����

		int fps;                                 // ���������� ������ � �������, ����������� �� ������. ���������� ���������� ������������ ��� � �������. ����� ����� ���������� �� ������.
		int selection_time;                      // ��������� ����� �������� ��������
		int curr_frame;                          // ����� �������� �����

		bool cut_shadows;                        // ������� ����, ����������� �� ������� ����� �������

		bool use_cuda;                           // ������������ CUDA ��� ��������� ���� � ����������
		int cuda_device_ind;                     // ������ ������������� ����������, ��������������� CUDA

        int pixel_size;                          // ������ ������ ������� � ������

		bool use_morphology;                     // ������������ �������� �������������� ���������� "��������" ��� ����������� ��������� ����

		int min_region_width;                    // ����������� ������ �
		int min_region_height;                   // ������ �������

		int left_padding;                        // ������ �� ������ �
		int top_padding;                         // �������� ���� ����� ��� ������� �����������
		RECT_ analyze_area;                      // ����� �����, ������� ����� ���������������

		bool need_background_update;             // ��������� �� ������ ������� �����

		regions_container regions;                                // ������ �������� �� ��������� �����
        void regions_preprocessing(const uchar* buf, uint32_t pitch); // ��������������� ������ � ��������� ��������: ��������� �����, �������� ��������� �������� � �.�.
#if !ADV_OUT
        void tracking_objects(const uchar* buf, uint32_t pitch);      // ������ �������� � ���������� ���������� �� �����
#else
        void tracking_objects(const uchar* buf, uint32_t pitch, uchar* adv_buf_rgb24); // ������ ��������
#endif

		objects_container objects_history;                    // ������ ��������, ��������� �� ���������� ������

        void add_uid_to_del_objects(unsigned int uid);               // ���������� �������������� � ������ ������� ��������
        void del_object(std::unique_ptr<CTrackingObject>& object, bool del_adv_data); // �������� ������� � ��������� � ��� ������
        std::vector<unsigned int> del_objects;                       // ������ ��������� ��������. ����������� ��� ��������� ���������������. ������� �� ������� ���������� �����
		size_t del_objects_count;                             // ���������� ��������� �������� �� ��������� �����

        unsigned int get_free_uid() const;                           // ��������� ���������� ������������� �������

		regions_container::iterator find_region_by_center(int c_x, int c_y, int width, int height); // ����� ����������� ������� �� ����������� ������ �������
        regions_container::iterator find_region_by_hist(const uchar* buf, int pitch, const std::unique_ptr<CTrackingObject>& obj);  // ����� ������� �� �����������

		float_t weight_threshold;                             // �����. ������� � ����� ���� ���� ���������� ��������������� ��� ����������� � ���������
		float_t weight_alpha;                                 // �����������, ���������� �� �������� �������� ���� ������� �� ������ ����

		std::vector<CObjRect> obj_rects;                      // ������ ��������� ��������. ����������� ��� ��������� ���������������. ������� �� ������� ���������� �����
		size_t objects_count;                                 // ���������� ��������� �������� �� ��������� �����

		std::list<CShadyLeftObj> shady_left_objects;          // �������, �������� ���������� ������������ ����������
		void add_to_shady_left_objects(CTrackingObject &obj); // ���������� ������� � ������ ��������, �������� ���������� ������������ ����������
        void del_from_shady_left_objects(unsigned int obj_uid);      // �������� ������� �� ������ ��������, �������� ���������� ������������ ����������
        void del_uid_from_shady_left_objects(unsigned int obj_uid);  // �������� �������������� ������� �� ������ ��������, �������� ���������� ������������ ����������
        void inc_time_shady_left_objects(unsigned int obj_uid);      // ���������� ������� ����� �������
		void analyze_shady_left_objects();                    // �������� ��������, �� ���������������� � ������� ���������� �������

		std::vector<CLeftObjRect> left_obj_rects;             // ������ ��������� ����������� ��������. ����������� ��� ��������� ���������������. ������� �� ������� ���������� �����
		size_t left_objects_count;                            // ���������� ����������� ��������
		bool show_left_objects;                               // �������� �� ����������� ����������� ��������

		bool show_objects;                                    // ���������� �������
		bool show_trajectory;                                 // ���������� ���������� ��������

		std::list<CLeftObjView> lefted_objects;               // ����������� ��������
		void analyze_lefted_objects();                        // ������ ����������� ���������

		lines_cont lines;                                     // ������ ���������������� ����� � ��������� ������������ �� �����
		lines_cont correct_lines;                             // ������ ���������������� ����� � ������������ ������������ ������� �������
		void recalc_correct_lines();                          // �������� ��������� ����� �� �����

        bool with_line_intersect(const CTrackingObject &obj, int new_center_x, int new_center_y); // �������� ����������� �������� �����

		zones_cont zones;                                     // ������ ��� �������� � ��������� ������������ �� �����
		zones_cont correct_zones;                             // ������ ��� �������� � ������������ ������������ ������� �������
		void recalc_correct_zones();                          // �������� ��������� ��� �� �����
		template<class T> bool is_in_zone(const T &rect, mstring *zone_name) const; // �������� �� ������������� � ������ ��� ��������
		template<class T> const CZone* get_zone(const T &rect) const;               // ��������� ����, � ������� �������� �������������

		template<class T> void add_object_to_out_rects(const T &rect, const CTrackingObject &object, object_types obj_type, const mstring &zone_name); // ���������� ������� �� �����
		void add_left_object_to_out_rects(const CLeftObjView &left_obj, CLeftObjRect::types type); // ���������� ������������ �������� �� �����

        bool use_recognition;                                // ������������ ������������� ��������

        bool cut_shadow(CObjectRegion& region);              // ��������� ����

		objects_container::iterator get_object_by_region(const CObjectRegion& region, objects_container::iterator from_obj); //��������� �������, �������� ������������ � ������� �������

        bgrnd_substr_types bgrnd_type;                       // ��� ��������� ��������� ����

        std::unique_ptr<CBackSubstraction> back_substractor; // ��������� ����
        CSegmentation segmentator;                           // ����������� �������� ��������� ����� � �������
        CRecognition recognizer;                             // ������������� ��������

	public:
		CFeinTrack();
		~CFeinTrack();

#if !ADV_OUT
        int new_frame(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, color_type buf_type); // ������ ���������� �����
#else
        int new_frame(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24); // ������ ���������� �����
#endif

		void set_sensitivity(int sens_level);                           // ����� ������� ���������������� ��� ��������� ���� (�� 1 �� 100)
		int get_sensitivity() const;                                    // ��������� ������ ���������������� ��� ��������� ���� (�� 1 �� 100)

		void get_objects(CObjRect* &rect_arr, size_t& rect_count);      // ��������� ������ ������������ ��������, ������������ �� ��������� �����
        void set_one_object(unsigned int uid, int left, int right, int top, int bottom); // ������� ��������� ������������� �������, ������� ����� ������������ �� �����
		bool get_object_points(size_t obj_ind, POINTF* points, size_t& max_points); // ��������� ������ �����, ������������� �������
        void get_del_objects(unsigned int* &uids_arr, size_t& uids_count);     // ��������� ������ ��������������� �������� �� ��������� ����� ��������
		void get_left_objects(CLeftObjRect* &rect_arr, size_t& rect_count); // ��������� ������ ����������� ��������

		int get_fps() const;                                            // ��������� �
		void set_fps(int new_fps);                                      // ������� fps

		void set_show_objects(bool new_val);                            // ����������/�� ���������� �������
		bool get_show_objects() const;                                  // ��������� ��������

		void set_zones_list(const zones_cont& zones_);                  // ������� �
		void get_zones_list(zones_cont& zones_) const;                  // ��������� ������ ���

		void set_show_left_objects(bool show_left_objects_);            // ������� �
		bool get_show_left_objects() const;                             // ��������� �������� show_left_objects

		void set_show_trajectory(bool show_trajectory_);                // ������� �
		bool get_show_trajectory() const;                               // ��������� �������� show_trajectory

		bool get_use_morphology() const;                                // ������� �
		void set_use_morphology(bool use_morphology_);                  // ��������� �������� use_morphology

		void set_lines_list(const lines_cont& lines_);                  // ������� �
		void get_lines_list(lines_cont& lines_) const;                  // ��������� ������ �����

		bool get_use_square_segmentation() const;                        // ��������� �
		void set_use_square_segmentation(bool use_square_segmentation_); // ������� ������ �����������

		bool get_use_recognition() const;                           // ��������� �
		void set_use_recognition(bool new_val);                     // ������� �������� use_recognition

		int get_min_region_width() const;                           // ��������� �
		void set_min_region_width(int min_region_width_);           // ������� ������������ ������� �������
		int get_min_region_height() const;                          // ��������� �
		void set_min_region_height(int min_region_height_);         // ������� ������������ ������� �������

		int get_selection_time() const;                             // ��������� �
		void set_selection_time(int selection_time_);               // ������� ������� ���������� ����� ��������

		RECT_ get_analyze_area() const;                             // ��������� �
		void set_analyze_area(const RECT_ &analyze_area_);          // ������� ������� ������� �����������

		int get_left_object_time1_sec() const;                      // ��������� �
		void set_left_object_time1_sec(int left_object_time1_sec_); // ������� ������� �������� ����������� ���������
		int get_left_object_time2_sec() const;                      // ��������� �
		void set_left_object_time2_sec(int left_object_time2_sec_); // ������� ������� �������� ����������� ���������
		int get_left_object_time3_sec() const;                      // ��������� �
		void set_left_object_time3_sec(int left_object_time3_sec_); // ������� ������� �������� ����������� ���������

		void enable_back_update(bool enable_val);                   // ����������/���������� ���������� ����

		bool get_detect_patches_of_sunlight() const;                           // ������������ �� �������� ������
		void set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_); // ���������/���������� ��������� ������

		bool get_cut_shadows() const;                               // ������������ �� ��������� �����
		void set_cut_shadows(bool cut_shadows_);                    // ���������/���������� ��������� �����

		void set_use_cuda(bool use_cuda_, int cuda_device_ind_);    // ������� �
		bool get_use_cuda() const;                                  // ��������� ������������� CUDA
		int get_cuda_device() const;                                // ��������� ������� ������������� ����������, ��������������� CUDA

		void set_bgrnd_type(bgrnd_substr_types bgrnd_type_);        // ������� �
		bgrnd_substr_types get_bgrnd_type() const;                  // ��������� ���� ������������� ��������� ��������� ����
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
