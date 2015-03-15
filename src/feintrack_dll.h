#pragma once

#include <string>
#include <deque>

#define ADV_OUT 1             // Вывод отладочной информации в отдельный буфер

#define USE_HOG_RECOGNIZE 0   // Распознавание людей с помощью OpenCV'шного HOG

typedef int int32_t;
typedef unsigned int uint32_t;
typedef unsigned char uchar;
typedef float float_t;

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
    ////////////////////////////////////////////////////////////////////////////

    // Строка
    typedef std::string cstring;
    ////////////////////////////////////////////////////////////////////////////

	// Точка
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

	// Простой прямоугольник
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

        unsigned int struct_size; // Размер структуры

		T left;          // Левая,
		T right;         // правая,
		T top;           // верхняя и
		T bottom;        // нижняя координаты прямоугольника

		T get_left() const { return left; }     // Функции, возвращающие границы прямоугольника
		T get_right() const { return right; }   //
		T get_top() const { return top; }       //
		T get_bottom() const { return bottom; } //

		T width() const { return right - left + 1; }      // Возвращает ширину
		T height() const { return bottom - top + 1; }     // и высоту прямоугольника

		int center_x() const { return (right + left) / 2; } // Возвращает центр прямоугольника по х
		int center_y() const { return (bottom + top) / 2; } // и по y

		bool operator==(const TRECT_ &rect)                  // Проверка на равенство двух прямоугольников
		{
			return (left == rect.left) && (right == rect.right) && (top == rect.top) && (bottom == rect.bottom);
		}
	};

	typedef TRECT_<int> RECT_;
	typedef TRECT_<double> RECTF_;
	////////////////////////////////////////////////////////////////////////////

	enum object_types                        // Типы объекта
	{
		unknown_object,                      // объект неизвестного типа
		human,                               // человек
		vehicle,                             // автомобиль
		animal,                              // животное
		humans                               // группа людей
	};
	////////////////////////////////////////////////////////////////////////////
	
	// Прямоугольник с координатами и идентификатором объекта
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

		object_types type;                       // Тип объекта

        unsigned int uid;                        // Уникальный идентификатор объекта

		int new_center_x;                        // Предположительное новое положение центра объекта
		int new_center_y;                        //

        std::string zone_name;                   // Имя зоны, в которой обнаружен объект (если объект обнаружен одновременно в 2-х зонах, то записывается имя только одной зоны)

        static const size_t MAX_TRAECTORY = 200; // Максимальный размер отображаемой траектории
		POINT_<int> traectory[MAX_TRAECTORY];    // Траектория движения объекта
		size_t traectory_size;                   // Текущий размер траектории
	};
	////////////////////////////////////////////////////////////////////////////
	
	// Прямоугольник с координатами оставленного предмета
	struct CLeftObjRect: public RECT_
	{
		enum types        // Типы объекта
		{
			first, second
		};
		types type;       // Тип объекта

		CLeftObjRect(const RECT_ &rect, types type_)
			: RECT_(rect), type(type_)
		{
			struct_size = sizeof(*this);
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// Поддерживаемые цветовые пространства
	enum color_type
	{
		unknown_color, // Неизвестный цвет
		buf_rgb24,     // 
		buf_gray,      // 
		buf_rgb32      // 
	};

	// Получение размера пикселя в байтах для различных типов цветовых пространств
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

    void* AddFeintrack(); // Создание
    void DelFeintrack(void* feintrack);                                           // и удаление FeinTracker'a

	// Подаёт новый кадр на обработку
#if !ADV_OUT
    int FeintrackFrameAnalyze(void* feintrack, const uchar* buf, int width, int height, color_type buf_type);
#else
    int FeintrackFrameAnalyze(void* feintrack, const uchar* buf, int width, int height, color_type buf_type, uchar* adv_buf_rgb24);
#endif

	// Получение конфигурации Feintrack'а
    bool GetFeintrackConfigStruct(void* feintrack, void* config_struct);
	// Задание конфигурации Feintrack'а
    void SetFeintrackConfigStruct(void* feintrack, const void* config_struct);
	// Изменение конфигурации Feintrack'а: задание нового uid и fps (т.к. они могут измениться при перезапуске Видеолокатора)
    void UpdateFeintrackConfigStruct(void* feintrack, int channel_fps, const char* channel_name, void* config_struct);

	// Используется ли детектирование объектов на Feintrack'e
    bool GetUseFeintrack(void* feintrack);
	// Использовать детектирование объектов в Feintrack'e
    void SetUseFeintrack(void* feintrack, bool new_val);

	// Получение списка объектов, обнаруженных на пoследнем кадре
    void GetObjects(void* feintrack, vl_feintrack::CObjRect* &rect_arr, size_t& rect_count);

	// Получение списка идентификаторов удалённых на последнем кадре объектов
    void GetDelObjects(void* feintrack, unsigned int* &uids_arr, size_t& uids_count);

	// Получение списка оставленных предметов
    void GetLeftObjects(void* feintrack, vl_feintrack::CLeftObjRect* &rect_arr, size_t& rect_count);

	// Разрешение/запрещение обновления фона в feintrack'e
    void EnableBackUpdate(void* feintrack, bool enable_val);

	// Активация профиля настроек finetrack'a
    bool ActivateProfile(void* feintrack, const char* profile_name);

	// Задание координат единственного объекта, который будет отображаться на кадре
    void SetOneObject(void* feintrack, unsigned int obj_uid, int left, int right, int top, int bottom);

	// Получение набора точек, принадлежащих объекту
    void GetObjectPoints(void* feintrack, size_t obj_ind, POINTF* points, size_t& max_points);
	////////////////////////////////////////////////////////////////////////////
} // end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
