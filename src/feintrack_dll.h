#pragma once

#include <string>
#include <deque>

#define ADV_OUT 1             // ����� ���������� ���������� � ��������� �����

#define USE_HOG_RECOGNIZE 0   // ������������� ����� � ������� OpenCV'����� HOG

typedef int int32_t;
typedef unsigned int uint32_t;
typedef unsigned char uchar;
typedef float float_t;

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
    ////////////////////////////////////////////////////////////////////////////

    // ������
    typedef std::string cstring;
    ////////////////////////////////////////////////////////////////////////////

	// �����
	template<class T>
	struct POINT_
	{
		POINT_(): x(0), y(0) {}
		POINT_(T x_, T y_): x(x_), y(y_) {}

		T x;
		T y;
	};

	typedef POINT_<float> POINTF;
	////////////////////////////////////////////////////////////////////////////

	// ������� �������������
	template <typename T>
	struct TRECT_
	{
		TRECT_(T left_, T right_, T top_, T bottom_)
			: left(left_), right(right_), top(top_), bottom(bottom_)
		{
			struct_size = sizeof(*this);
		}
		TRECT_()
			: left(0), right(0), top(0), bottom(0)
		{
			struct_size = sizeof(*this);
		}

        unsigned int struct_size; // ������ ���������

		T left;          // �����,
		T right;         // ������,
		T top;           // ������� �
		T bottom;        // ������ ���������� ��������������

		T get_left() const { return left; }     // �������, ������������ ������� ��������������
		T get_right() const { return right; }   //
		T get_top() const { return top; }       //
		T get_bottom() const { return bottom; } //

		T width() const { return right - left + 1; }      // ���������� ������
		T height() const { return bottom - top + 1; }     // � ������ ��������������

		int center_x() const { return (right + left) / 2; } // ���������� ����� �������������� �� �
		int center_y() const { return (bottom + top) / 2; } // � �� y

		bool operator==(const TRECT_ &rect)                  // �������� �� ��������� ���� ���������������
		{
			return (left == rect.left) && (right == rect.right) && (top == rect.top) && (bottom == rect.bottom);
		}
	};

	typedef TRECT_<int> RECT_;
	typedef TRECT_<double> RECTF_;
	////////////////////////////////////////////////////////////////////////////

	enum object_types                        // ���� �������
	{
		unknown_object,                      // ������ ������������ ����
		human,                               // �������
		vehicle,                             // ����������
		animal,                              // ��������
		humans                               // ������ �����
	};
	////////////////////////////////////////////////////////////////////////////
	
	// ������������� � ������������ � ��������������� �������
	struct CObjRect: public RECT_
	{
        CObjRect(int left_, int right_, int top_, int bottom_, unsigned int uid_, int new_center_x_, int new_center_y_)
			: RECT_(left_, right_, top_, bottom_),
            type(unknown_object), uid(uid_),
            new_center_x(new_center_x_), new_center_y(new_center_y_),
			traectory_size(0)
		{
			zone_name[0] = '\0';

			traectory[0].x = center_x();
			traectory[0].y = center_y();
			traectory_size = 1;

			struct_size = sizeof(*this);
		}

		object_types type;                       // ��� �������

        unsigned int uid;                        // ���������� ������������� �������

		int new_center_x;                        // ����������������� ����� ��������� ������ �������
		int new_center_y;                        //

        std::string zone_name;                   // ��� ����, � ������� ��������� ������ (���� ������ ��������� ������������ � 2-� �����, �� ������������ ��� ������ ����� ����)

        static const size_t MAX_TRAECTORY = 200; // ������������ ������ ������������ ����������
		POINT_<int> traectory[MAX_TRAECTORY];    // ���������� �������� �������
		size_t traectory_size;                   // ������� ������ ����������
	};
	////////////////////////////////////////////////////////////////////////////
	
	// ������������� � ������������ ������������ ��������
	struct CLeftObjRect: public RECT_
	{
		enum types        // ���� �������
		{
			first, second
		};
		types type;       // ��� �������

		CLeftObjRect(const RECT_ &rect, types type_)
			: RECT_(rect), type(type_)
		{
			struct_size = sizeof(*this);
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// �������������� �������� ������������
	enum color_type
	{
		unknown_color, // ����������� ����
		buf_rgb24,     // 
		buf_gray,      // 
		buf_rgb32      // 
	};

	// ��������� ������� ������� � ������ ��� ��������� ����� �������� �����������
	template<class T> inline
        T get_pixel_size(color_type cl_type)
	{
		switch (cl_type)
		{
		case unknown_color: return 0;
		case buf_rgb24: return 3;
		case buf_gray: return 1;
		case buf_rgb32: return 4;
		default: return 0;
		}
    }
	////////////////////////////////////////////////////////////////////////////

    void* AddFeintrack(); // ��������
    void DelFeintrack(void* feintrack);                                           // � �������� FeinTracker'a

	// ����� ����� ���� �� ���������
#if !ADV_OUT
    int FeintrackFrameAnalyze(void* feintrack, const uchar* buf, int width, int height, color_type buf_type);
#else
    int FeintrackFrameAnalyze(void* feintrack, const uchar* buf, int width, int height, color_type buf_type, uchar* adv_buf_rgb24);
#endif

	// ��������� ������������ Feintrack'�
    bool GetFeintrackConfigStruct(void* feintrack, void* config_struct);
	// ������� ������������ Feintrack'�
    void SetFeintrackConfigStruct(void* feintrack, const void* config_struct);
	// ��������� ������������ Feintrack'�: ������� ������ uid � fps (�.�. ��� ����� ���������� ��� ����������� �������������)
    void UpdateFeintrackConfigStruct(void* feintrack, int channel_fps, const char* channel_name, void* config_struct);

	// ������������ �� �������������� �������� �� Feintrack'e
    bool GetUseFeintrack(void* feintrack);
	// ������������ �������������� �������� � Feintrack'e
    void SetUseFeintrack(void* feintrack, bool new_val);

	// ��������� ������ ��������, ������������ �� �o������� �����
    void GetObjects(void* feintrack, vl_feintrack::CObjRect* &rect_arr, size_t& rect_count);

	// ��������� ������ ��������������� �������� �� ��������� ����� ��������
    void GetDelObjects(void* feintrack, unsigned int* &uids_arr, size_t& uids_count);

	// ��������� ������ ����������� ���������
    void GetLeftObjects(void* feintrack, vl_feintrack::CLeftObjRect* &rect_arr, size_t& rect_count);

	// ����������/���������� ���������� ���� � feintrack'e
    void EnableBackUpdate(void* feintrack, bool enable_val);

	// ��������� ������� �������� finetrack'a
    bool ActivateProfile(void* feintrack, const char* profile_name);

	// ������� ��������� ������������� �������, ������� ����� ������������ �� �����
    void SetOneObject(void* feintrack, unsigned int obj_uid, int left, int right, int top, int bottom);

	// ��������� ������ �����, ������������� �������
    void GetObjectPoints(void* feintrack, size_t obj_ind, POINTF* points, size_t& max_points);
	////////////////////////////////////////////////////////////////////////////
} // end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
