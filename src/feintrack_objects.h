#pragma once
#include <deque>
#include <memory>

#include "feintrack_dll.h"
#include "utils.h"
////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	//������������ �������� ������������ ��������� ������� (����� ������������ ����� �������� �������� �� � ��������������� ���������� ���������� ���������� �������-������)
#define LIN_MNK 0
	////////////////////////////////////////////////////////////////////////////
	
	// ������ ���������� � �������, �������������� ������
	class CObjectRegion
	{
	private:
		int left;             //�����,
		int right;            //������,
		int top;              //�������
		int bottom;           //� ������ ���������� ��������������, ��������������� ������

		size_t obj_pxl_count; //���������� �������� ������� � �������

		static const int region_h_granz = 3; //������� ������� ������� ����� � ������
		static const int region_v_granz = 4; //������� ������� ������� ������ � �����

	public:
		CObjectRegion();
		CObjectRegion(int left_, int right_, int top_, int bottom_, size_t obj_pxl_count_ = 1);
		CObjectRegion(const CObjectRegion& region);
		~CObjectRegion();

		int get_left() const;               //�������, ������������ ������� �������
		int get_right() const;              //
		int get_top() const;                //
		int get_bottom() const;             //

		void set_left(int left_);           //�������, �������� ������� �������
		void set_right(int right_);         //
		void set_top(int top_);             //
		void set_bottom(int bottom_);       //

		int width() const;                  // ���������� ������
		int height() const;                 // � ������ ��������������, ��������������� ������

		//��������: ����� �� ����� � ���������� ������������ ������ �������, �� ��� ����� � ��� ���������
		bool in_region(int x, int y) const;

		//���� ����� ������� ����� � ���������� ������������ �� ����������� �������, �� �� ����������� ���� �������
		void add_point(int x, int y);

		void regions_merging(const CObjectRegion& region); //����������� 2-� ��������

		//��������: ����� �� ������������� � ���������� ������������ ����� � ��������
		bool near_region(int left_, int top_) const;

		//���������� �������������� � ������� (����������� ������������� ��������� ���� ������, ���� �����)
		void add_rect(int right_, int bottom_, size_t obj_pxl_count_);

		int get_center_x() const;           //���������� ���������� ������ �������
		int get_center_y() const;           //

		float_t density() const;            //���������� ��������� ����� ������� � ������� � ������ ���������� �����

		void resize_to_max_granz(int max_width, int max_height); //��������� ���� ������� � ������ �������� region_h_granz � region_v_granz

		static bool size_bigger(const CObjectRegion& reg1, const CObjectRegion& reg2); //������� ��� ���������� �������� �� �������
	};
	////////////////////////////////////////////////////////////////////////////
	
	// ������ ���������� �� ��������, ���������� �� ���������� ������
	class CTrackingObject
	{
	private:
		typedef std::deque<POINT_<int> > traectory_cont;

		int center_x;                                     //���������� ���������� ������ �������
		int center_y;                                     //

		RECT_ rect;                                       //������������� � ���������� ��������� �������

		int dx;                                           //�������� ���������� ������� ������� �� �����������
		int dy;                                           //� �� ���������

		int left_center_x;                                //��������� ���������� ������ ������������ ��������
		int left_center_y;                                //
		int frames_in_eps;                                //���������� ������, �� ������� ����� ������� �� ����� �� ������� left_eps
		int left_epsilon;                                 //������������ �������� �������� ����� �������, ��� ���������� ������� ������ �������� ���� �����������

		int obj_recogn_count;                             //���������� ������������� ������������� ������� ��� ��������
		object_types type;                                //������� ��� �������
		static const int min_recogn_count = 5;            //����������� ���������� ������������� �������������, ��� ������� ������ ��������� � ������� ����

		CTrackingObject *merge_object1;                   //�������,
		CTrackingObject *merge_object2;                   //� �������� ���� ����������� ������� ��� ����������� ����������

#if LIN_MNK
		static const size_t STAT_FRAME_COUNT = 24;        //���������� ������, ������� ������������ ��� ������������ �������� ��������� �������
		cyclic_array<POINT_<int>, STAT_FRAME_COUNT> stat; //���������� ������� �� ���������� ������
		size_t coords_collected;                          //���������� ��� ��������� ���������
		float_t kx, bx, ky, by;                           //������������ ��������� ���������� �������� �������
#else
		static const int dp_epsilon = 5;                  // �������� ������ ��������� �������-������ (� ��������)
		traectory_cont dp_traectory_x;                    // ���������� �������� ������� �� ����������� �
		traectory_cont dp_traectory_y;                    // �� ��������� ����� ��������� � ���������� �������-������

		int predict(const traectory_cont& traectory, const traectory_cont& orig_traectory, int delta_time) const; // ������������ ��������� ������� �� ��������� ���������� ������� � ������
#endif

		traectory_cont traectory_x;                       //������ ���������� �������� ������� �� ����������� �
		traectory_cont traectory_y;                       //�� ���������

	public:
        CTrackingObject(int center_x_, int center_y_, unsigned int uid_);
		CTrackingObject(const CTrackingObject &obj);
		~CTrackingObject();

		object_types get_type() const;                    //��������� ���� �������
		void set_new_type(object_types new_type);         //������� ������������� �� ��������� ����� ���� �������

		bool have_merge_object() const;                          //���� �� ������, � ������� ����������� �������
		void add_merge_obj(const CTrackingObject &merge_object); //������� � ��������
		CTrackingObject *get_merge_object(size_t ind);           //��������� �������, � ������� ���� ����������� �������
		void inc_merge_frames();                                 //���������� ������� ������� �� 1
		void set_merge_objects_to_null();                        //�������� ��������� �� �������
        bool has_merge_object(unsigned int object_uid) const;           //������� �� ������ � ������� uid

		static const float_t default_weight;              //�������������� �������� ���� �������

		int get_x_future_val(int future_time) const;      //��������� ������ ���������� x ������ ������ ������� ����� future_time ������
		int get_y_future_val(int future_time) const;      //��������� ������ ���������� y ������ ������ ������� ����� future_time ������

		int get_new_center_x() const;                     //��������� ��������������� ������ ���������� x ������ ������ ������� �� ��������� ��������� ������
		int get_new_center_y() const;                     //��������� ��������������� ������ ���������� y ������ ������ ������� �� ��������� ��������� ������

		int get_last_center_x() const;                    //��������� ��������� ���������� x ������ �������
		int get_last_center_y() const;                    //��������� ��������� ���������� y ������ �������

		int get_left_frames() const;                      //��������� �
		void set_left_frames(int frames_in_eps_);         //������� ���������� ������, �� ������� �������� �������� ������� ������ ��������
		int get_left_epsilon() const;                     //���������� ����������� ��������� �������� ���������� ������ ������� ��� ����������� ����������� ���������

		void set_last_center(int new_center_x, int new_center_y); //������� ��������� ���������� ��������������� ������ �������
		void set_rect(int left, int right, int top, int bottom);  //������� ����� ��������� �������

		//��������� ��������� ������������� ��������� �������
		const RECT_ &get_rect() const;
		int get_left() const;
		int get_right() const;
		int get_top() const;
		int get_bottom() const;

		int width() const;  //��������� ������ �
		int height() const; //������ �������

		void recalc_center(); //������������� ���������� ������ ������� ��� ���������� ���������� � ��� �������������� �� ��������� �����

		void get_traectory(CObjRect &obj_rect, uint frame_width, uint frame_height, int left_padding, int top_padding) const; //��������� ���������� �������� �������

        unsigned int uid;            //������������� (�����) �������

		float_t weight;       //��� - ��������, ������� �� ���������� (0; 1) � ������������ ������������ ������� �������
		int life_time;        //����� ����� ������� � ������
		int frames_left;      //���������� ������, �� ������� ������ �� ��� ������

		//������� ��� ���������� �������� �� ����, ������� ����� � ������������ (��� + ������� �����) ����������
		static bool weight_bigger(const CTrackingObject &obj1, const CTrackingObject &obj2);
        static bool life_bigger(const std::unique_ptr<CTrackingObject>& obj1, const std::unique_ptr<CTrackingObject>& obj2);
		static bool weight_life_bigger(const CTrackingObject &obj1, const CTrackingObject &obj2);
	};
	////////////////////////////////////////////////////////////////////////////
	
	// ������ ���������� �� ����������� ��������
	struct CLeftObjView
	{
		CLeftObjView()
			: life_time(0)
		{}
		CLeftObjView(int life_time_, int left, int right, int top, int bottom)
			: life_time(life_time_)
		{
			rect.left = left;
			rect.right = right;
			rect.top = top;
			rect.bottom = bottom;
		}
		int life_time; //����� ����� ��������
		RECT_ rect;    //������������� ��������

		//��������� �� ������� �����
		static bool bigger(const CLeftObjView &obj1, const CLeftObjView &obj2)
		{
			return obj1.life_time > obj2.life_time;
		}
	};
	////////////////////////////////////////////////////////////////////////////
	
	// ������, �������� ���������� �����������
	struct CShadyLeftObj: public CLeftObjView
	{
		CShadyLeftObj()
			: real_obj_uid(0), not_detect_time(0)
		{
		}
        CShadyLeftObj(unsigned int real_obj_uid_, int life_time_, int left, int right, int top, int bottom)
			: CLeftObjView(life_time_, left, right, top, bottom), real_obj_uid(real_obj_uid_), not_detect_time(0)
		{
		}
        unsigned int real_obj_uid;  //������������� �������, ������� �������� �������� �����������
		int not_detect_time; //���������� ������, �� ������� ������ �� ��� ������
	};
	////////////////////////////////////////////////////////////////////////////

	typedef std::list<CObjectRegion> regions_container;
    typedef std::list<std::unique_ptr<CTrackingObject> > objects_container;
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
