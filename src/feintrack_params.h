#pragma once

#include <string>
#include <deque>

#include "feintrack_d.h"

#define ADV_OUT 1             // Вывод отладочной информации в отдельный буфер

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
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
        }
        TRECT_()
            : left(0), right(0), top(0), bottom(0)
        {
        }

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

    // Прямоугольник с координатами и идентификатором объекта
    struct CObjRect: public RECT_
    {
        CObjRect()
            :
              uid(0),
              new_center_x(0), new_center_y(0),
              trajectory_size(0)
        {
            zone_name[0] = '\0';

            trajectory[0].x = center_x();
            trajectory[0].y = center_y();
            trajectory_size = 1;
        }

        CObjRect(int left_, int right_, int top_, int bottom_, unsigned int uid_, int new_center_x_, int new_center_y_)
            : RECT_(left_, right_, top_, bottom_),
            uid(uid_),
            new_center_x(new_center_x_), new_center_y(new_center_y_),
            trajectory_size(0)
        {
            zone_name[0] = '\0';

            trajectory[0].x = center_x();
            trajectory[0].y = center_y();
            trajectory_size = 1;
        }

        unsigned int uid;                        // Уникальный идентификатор объекта

        int new_center_x;                        // Предположительное новое положение центра объекта
        int new_center_y;                        //

        std::string zone_name;                   // Имя зоны, в которой обнаружен объект (если объект обнаружен одновременно в 2-х зонах, то записывается имя только одной зоны)

        static const size_t MAX_TRAECTORY = 200; // Максимальный размер отображаемой траектории
        POINT_<int> trajectory[MAX_TRAECTORY];    // Траектория движения объекта
        size_t trajectory_size;                   // Текущий размер траектории
    };
    ////////////////////////////////////////////////////////////////////////////

    // Прямоугольник с координатами оставленного предмета
    struct CLeftObjRect: public RECT_
    {
        enum types             // Типы объекта
        {
            first, second
        };
        types type;            // Тип объекта
        unsigned int obj_uid;  // Идентификатор объекта, который стал оставленным

        CLeftObjRect(const RECT_ &rect, types type_, unsigned int obj_uid_)
            : RECT_(rect), type(type_), obj_uid(obj_uid_)
        {
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

	// Зона детекции
	struct CZone: public RECT_
	{
        CZone(int left_, int right_, int top_, int bottom_, unsigned int uid_)
            :
              RECT_(left_, right_, top_, bottom_),
              uid(uid_),
              min_obj_width(1),
              min_obj_height(1),
              use_detection(false)
        {
        }
		CZone()
            :
              RECT_(),
              uid(0),
              min_obj_width(1),
              min_obj_height(1),
              use_detection(true)
		{
		}

        unsigned int uid;   // Уникальный идентификатор зоны
        std::string name;   // Имя зоны

		int min_obj_width;  // Минимальная ширина и
		int min_obj_height; // высота объекта, который может детектироваться в данной зоне

		bool use_detection; // Использовать детекцию на данной зоне

		// Равенство 2-х зон определяется равенством их uid
		bool operator==(const CZone &zone) const
		{
			return zone.uid == uid;
		}

		bool in_zone(int x, int y) const //Находится ли точка с указанными координатами внутри зоны
		{
			return ((left <= x) && (right >= x)) && ((top <= y) && (bottom >= y));
		}
	};
    typedef std::deque<CZone> zones_cont;
	////////////////////////////////////////////////////////////////////////////

	// Разделительная линия
	struct CLine
	{
		CLine()
			: x1(0), y1(0), x2(0), y2(0), uid(0)
		{
		}

		int x1;       // Координаты начальной точки линии
		int y1;       //

		int x2;       // Координаты конечной точки линии
		int y2;       //

        unsigned int uid;    // Уникальный идентификатор линии
        std::string name;    // Имя линии

		// Равенство 2-х линий определяется равенством их uid
		bool operator==(const CLine &line) const
		{
			return line.uid == uid;
		}
	};
    typedef std::deque<CLine> lines_cont;
	////////////////////////////////////////////////////////////////////////////

	// Типы поддерживаемых алгоритмов вычитания фона
	enum bgrnd_substr_types
	{
		norm_back = 0,       // Моделирование каждого пикселя нормально распределённым случайным вектором
		gaussian_mixture     // Моделирование каждого пикселя смесью нормально распределённых случайных векторов
	};
	////////////////////////////////////////////////////////////////////////////
    struct VideoHeader
    {
        const uchar* buf;
        uint32_t pitch;
        uint32_t frame_width;
        uint32_t frame_height;
        int left_padding;
        int top_padding;
        RECT_ analyze_area;
        int fps;
        int pixel_size;

#if ADV_OUT
        uchar* adv_buf_rgb24;
#endif

        VideoHeader()
            :
              buf(nullptr),
              pitch(0),
              frame_width(0),
              frame_height(0),
              left_padding(0),
              top_padding(0),
              fps(0),
              pixel_size(0)
#if ADV_OUT
              , adv_buf_rgb24(nullptr)
#endif
        {

        }
    };

    ////////////////////////////////////////////////////////////////////////////
} // end namespace feintrack
////////////////////////////////////////////////////////////////////////////
