#pragma once
#include <algorithm>
#include <limits>
#include <vector>
#include <string>
#include <list>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <assert.h>

#include "feintrack_params.h"

////////////////////////////////////////////////////////////////////////////

namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////

    // Функция конвертации BGR -> яркость
    template<class T> inline
    T RGB_2_Y(T R, T G, T B)
    {
        return T(0.3 * R + 0.59 * G + 0.11 * B);
    }
    template<class T> inline
    T RGB_2_Y(const T *buf)
    {
        return RGB_2_Y(buf[2], buf[1], buf[0]);
    }
    ////////////////////////////////////////////////////////////////////////
	
	// Циклический вектор
	template<class T, size_t SIZE>
	class cyclic_array
	{
	private:
        T v[SIZE];

		size_t pos;

		void inc_pos()
		{
			++pos;
			pos %= SIZE;
		}
	public:
		cyclic_array()
			: pos(0)
		{
			memset(v, 0, SIZE * sizeof(T));
		}
		cyclic_array(const cyclic_array &ca)
		{
			pos = ca.pos;
			for (size_t i = 0; i < SIZE; ++i)
			{
				v[i] = ca.v[i];
			}
		}

		void clear()
		{
			pos = 0;
			memset(v, 0, SIZE * sizeof(T));
		}

		T &operator[](size_t ind)
		{
			return v[(pos + ind) % SIZE];
		}
		const T &operator[](size_t ind) const
		{
			return v[(pos + ind) % SIZE];
		}

		size_t size() const
		{
			return SIZE;
		}

		void shift_last_elem(const T &new_val)
		{
			inc_pos();
			(*this)[size() - 1] = new_val;
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// Предикаты для сортировки точек по каждой из координат
	template<class POINT_TYPE>
	bool point_x_pred(const POINT_TYPE& p1, const POINT_TYPE& p2)
	{
		return p1.x < p2.x;
	}
	template<class POINT_TYPE>
	bool point_y_pred(const POINT_TYPE& p1, const POINT_TYPE& p2)
	{
		return p1.y < p2.y;
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Возвращает знак выражения
	template<class T> inline
	T sign(T val)
	{
		return (val < T(0))? T(-1): T(1);
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Возведение в квадрат
	template<class T> inline
	T sqr(T val)
	{
		return val * val;
	}
	////////////////////////////////////////////////////////////////////////
	
	// Возвращает расстояние между точками
	template<class T, class POINT_TYPE> inline
	T distance(const POINT_TYPE& p1, const POINT_TYPE& p2)
	{
		return sqrt((T)(sqr(p2.x - p1.x) + sqr(p2.y - p1.y)));
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Возвращает длину вектора
	template<class T> inline
	T v_length(T x, T y)
	{
		return sqrt(sqr(x) + sqr(y));
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Двумерный вектор
	template<class T>
	struct vector2
	{
		T x;    // 1-я и
		T y;    // 2-я координаты вектора
		T len;  // Длина вектора

		vector2(T x_, T y_)
			: x(x_), y(y_)
		{
			len = v_length(x, y);
		}
		vector2()
			: x(0), y(0), len(0)
		{
		}
	};
	////////////////////////////////////////////////////////////////////////////
	
	// Возвращает скалярное произведение векторов
	template<class T, class VECTOR_TYPE> inline
	T dot_product(const VECTOR_TYPE &v1, const VECTOR_TYPE &v2)
	{
		return (T)(v1.x * v2.x + v1.y * v2.y);
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Преобразование двумерных декартовых координат в координаты окна и обратно
	inline float_t wnd_to_x(int i, int wnd_width, float_t a, float_t b)
	{
		return a + (i * (b - a)) / wnd_width;
	}
	////////////////////////////////////////////////////////////////////////////
	inline int x_to_wnd(float_t x, int wnd_width, float_t a, float_t b)
	{
		return (int)(((x - a) * wnd_width) / (b - a));
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Проверка наложения друг на друга параллельных отрезков
	template<class T> inline
	bool segments_superposition(T a1, T a2, T b1, T b2)
	{
		if (a1 <= b1) return a2 >= b1;
		else return a1 <= b2;
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Перевод из одной прямоугольной системы координат в другую
	template<class T> inline
	T from_to(T val, T a_to, T b_to, T c_from, T d_from)
	{
		return a_to + ((val - c_from) * (b_to - a_to)) / (d_from - c_from);
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Ограничивает значение указанными рамками
	template<class T> inline
	T set_range(T &val, T min_val, T max_val)
	{
		if (val < min_val)
			val = min_val;
		else
			if (val > max_val)
				val = max_val;
		return val;
	}
	template<class T> inline
	T set_cycle_range(T &val, T min_val, T max_val)
	{
		if (val < min_val)
			val += max_val;
		else
			if (val > max_val)
				val -= max_val;
		return val;
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Проверка значения на принадлежность отрезку
	template<class T> inline
	bool in_range(T curr_val, T min_val, T max_val)
	{
		return ((curr_val >= min_val) && (curr_val <= max_val));
	}
	////////////////////////////////////////////////////////////////////////////
	
	// Возвращает true, если отрезки, заданные координатами конечных точек в однородных декартовых координатах, пересекаются
	bool is_intersect(float_t x1_1, float_t y1_1, float_t x2_1, float_t y2_1,
		              float_t x1_2, float_t y1_2, float_t x2_2, float_t y2_2);
	////////////////////////////////////////////////////////////////////////////

	// Получение площади пересечения прямоугольников
	template<class T, class SOME_RECT>
	T get_intersect_area(const SOME_RECT &r1, const SOME_RECT &r2)
	{
		int w = 0;
		int h = 0;

		if (r1.x <= r2.x)
		{
			if (r1.x + r1.width >= r2.x)
				w = std::min(r1.x + r1.width, r2.x + r2.width) - r2.x;
		}
		else
		{
			if (r1.x <= r2.x + r2.width)
				w = std::min(r1.x + r1.width, r2.x + r2.width) - r1.x;
		}

		if (w)
		{
			if (r1.y <= r2.y)
			{
				if (r1.y + r1.height >= r2.y)
					h = std::min(r1.y + r1.height, r2.y + r2.height) - r2.y;
			}
			else
			{
				if (r1.y <= r2.y + r2.height)
					h = std::min(r1.y + r1.height, r2.y + r2.height) - r1.y;
			}
		}

		return (T)(w * h);
	}
	////////////////////////////////////////////////////////////////////////////

	template<class T>
	T get_intersect_area(const RECT_& source, const RECT_& dest)
	{
		int x_coord[4] = { source.left, source.right, dest.left, dest.right };
		int y_coord[4] = { source.top, source.bottom, dest.top, dest.bottom };

		// x1 x2 - source rect, x3 x4 - dest rect
		if ((x_coord[2] > x_coord[1] || x_coord[0] > x_coord[3])  ||
			(y_coord[2] > y_coord[1] || y_coord[0] > y_coord[3]))
			return 0.0;

        std::sort(x_coord, x_coord + 4);
        std::sort(y_coord, y_coord + 4);

		return (T)((x_coord[2] - x_coord[1]) * (y_coord[2] - y_coord[1]));
	}
	////////////////////////////////////////////////////////////////////////////

	//Получение уравнения линейной регресии методом наименьших квадратов
	template<class CONT>
	void get_lin_regress_params(const CONT& in_data, size_t start_pos, size_t in_data_size, double &kx, double &bx, double &ky, double &by)
	{
		double m1(0.), m2(0.);
		double m3_x(0.), m4_x(0.);
		double m3_y(0.), m4_y(0.);

		const double el_count = (double)(in_data_size - start_pos);
		for (size_t i = start_pos; i < in_data_size; ++i)
		{
			m1 += i;
			m2 += sqr(i);
			
			m3_x += in_data[i].x;
			m4_x += i * in_data[i].x;

			m3_y += in_data[i].y;
			m4_y += i * in_data[i].y;
		}

		double det_1 = 1. / (el_count * m2 - sqr(m1));

		m1 *= -1.;

		kx = det_1 * (m1 * m3_x + el_count * m4_x);
		bx = det_1 * (m2 * m3_x + m1 * m4_x);

		ky = det_1 * (m1 * m3_y + el_count * m4_y);
		by = det_1 * (m2 * m3_y + m1 * m4_y);
	}
	////////////////////////////////////////////////////////////////////////////
	
	//Получение уравнения линейной регресии адаптивным методом наименьших квадратов
	template<class CONT>
	void get_lin_regress_params_a(const CONT& in_data, size_t start_pos, size_t in_data_size, double &kx, double &bx, double &ky, double &by)
	{
		double m1(0.), m2(0.);
		double m3_x(0.), m4_x(0.);
		double m3_y(0.), m4_y(0.);

		const double el_count = (double)(in_data_size - start_pos);
		double j = 1.;
		const double ids_1 = 1. / el_count;
		double tmp(0.);
		for (size_t i = start_pos; i < in_data_size; ++i, j += 2.)
		{
			tmp = j * ids_1;

			m1 += tmp * i;
			m2 += tmp * sqr(i);
			
			m3_x += tmp * in_data[i].x;
			m4_x += i * tmp * in_data[i].x;

			m3_y += tmp * in_data[i].y;
			m4_y += i * tmp * in_data[i].y;
		}

		double det_1 = 1. / (el_count * m2 - sqr(m1));

		m1 *= -1.;

		kx = det_1 * (m1 * m3_x + el_count * m4_x);
		bx = det_1 * (m2 * m3_x + m1 * m4_x);

		ky = det_1 * (m1 * m3_y + el_count * m4_y);
		by = det_1 * (m2 * m3_y + m1 * m4_y);
	}
	////////////////////////////////////////////////////////////////////////////
	
	//Аппроксимация кривой, заданной рядом точек, квадратичной функцией методом наименьших квадратов
	template<class CONT>
	void mnk_parabola(const CONT& in_data, size_t start_pos, size_t in_data_size,
		double &c1_x, double &c2_x, double &c3_x, double &c1_y, double &c2_y, double &c3_y)
	{
		double b1_x(0), b2_x(0), b3_x(0);
		double b1_y(0), b2_y(0), b3_y(0);
		double t_0(0.), t_1(0.), t_2(0.), t_3(0.), t_4(0.);
		double j(start_pos);
		for (size_t i = start_pos; i < in_data_size; ++i, j += 1.)
		{
			double sqr_j = sqr(j);

			t_0 += 1.;
			t_1 += j;
			t_2 += sqr_j;
			t_3 += j * sqr_j;
			t_4 += sqr(sqr_j);

			b1_x += in_data[i].x;
			b2_x += j * in_data[i].x;
			b3_x += sqr_j * in_data[i].x;

			b1_y += in_data[i].y;
			b2_y += j * in_data[i].y;
			b3_y += sqr_j * in_data[i].y;
		}

		//Система линейных неоднородных уравнений 3х3. Решаем методом Крамера
		double a11(t_0), a12(t_1), a13(t_2), a21(t_1), a22(t_2), a23(t_3), a31(t_2), a32(t_3), a33(t_4);

		double det_1 = 1. / (a11 * a22 * a33 + a21 * a32 * a13 + a12 * a23 * a31 - a31 * a22 * a13 - a11 * a23 * a32 - a12 * a21 * a33);
		c1_x = det_1 * (b1_x * a22 * a33 + b2_x * a32 * a13 + a12 * a23 * b3_x - b3_x * a22 * a13 - b1_x * a23 * a32 - a12 * b2_x * a33);
		c2_x = det_1 * (a11 * b2_x * a33 + a21 * b3_x * a13 + b1_x * a23 * a31 - a31 * b2_x * a13 - a11 * a23 * b3_x - b1_x * a21 * a33);
		c3_x = det_1 * (a11 * a22 * b3_x + a21 * a32 * b1_x + a12 * b2_x * a31 - a31 * a22 * b1_x - a11 * b2_x * a32 - a12 * a21 * b3_x);
		c1_y = det_1 * (b1_y * a22 * a33 + b2_y * a32 * a13 + a12 * a23 * b3_y - b3_y * a22 * a13 - b1_y * a23 * a32 - a12 * b2_y * a33);
		c2_y = det_1 * (a11 * b2_y * a33 + a21 * b3_y * a13 + b1_y * a23 * a31 - a31 * b2_y * a13 - a11 * a23 * b3_y - b1_y * a21 * a33);
		c3_y = det_1 * (a11 * a22 * b3_y + a21 * a32 * b1_y + a12 * b2_y * a31 - a31 * a22 * b1_y - a11 * b2_y * a32 - a12 * a21 * b3_y);
	}
	////////////////////////////////////////////////////////////////////////////

	// Вычисление высоты треугольника, опущенной из вершины b на сторону ac
	template<class T, class POINT_TYPE> inline
		T h_tr(const POINT_TYPE& a, const POINT_TYPE& b, const POINT_TYPE& c)
	{
		return abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)) / distance<T>(a, c);
	}
	////////////////////////////////////////////////////////////////////////////

	// Алгоритм Дугласа-Пекера
	template<class CONT, class ITERATOR>
	void DouglasPeucker(const ITERATOR& in_begin, const ITERATOR& in_end, CONT& points_out, int epsilon)
	{
		int dmax = 0;
		ITERATOR a(in_begin);
		ITERATOR c(in_end);
		ITERATOR b(in_begin);

		for (ITERATOR i = in_begin + 1; i != c; ++i)
		{
			int d = (int)h_tr<double>(*a, *i, *c);
			if (d > dmax)
			{
				b = i;
				dmax = d;
			}
		}

		if (dmax >= epsilon)
		{
			CONT points_out1;
			DouglasPeucker(a, b, points_out1, epsilon);
			CONT points_out2;
			DouglasPeucker(b, c, points_out2, epsilon);

			points_out.insert(points_out.begin(), points_out1.begin(), points_out1.end());
			points_out.insert(points_out.end(), points_out2.begin() + 1, points_out2.end());
		}
		else
		{
			points_out.push_back(*a);
			points_out.push_back(*c);
		}
	}
	////////////////////////////////////////////////////////////////////////////

	typedef std::vector<double> hist_cont;
	////////////////////////////////////////////////////////////////////////////

	// Построение гистограммы по строкам
	template<class SOME_RECT, class MASK_TYPE>
    void build_horz_hist(const SOME_RECT& region, int from_h, int to_h, hist_cont& hist, uint32_t frame_width, const MASK_TYPE* mask)
	{
		int w = frame_width - region.width();
		const MASK_TYPE* par = mask + region.get_left() + frame_width * (region.get_top() + from_h);

		for (int j = from_h; j < to_h; ++j)
		{
			for (int i = 0; i < region.width(); ++i)
			{
				if (*par)
					hist[i]++;

				++par;
			}
			par += w;
		}
	}
	////////////////////////////////////////////////////////////////////////////

	// Построение гистограммы по столбцам	
	template<class SOME_RECT, class MASK_TYPE>
    void build_vert_hist(const SOME_RECT& region, hist_cont& hist, uint32_t frame_width, const MASK_TYPE* mask)
	{
		int w = frame_width - region.width();
		const MASK_TYPE *par = mask + region.get_left() + frame_width * region.get_top();

		for (int j = 0; j < region.height(); ++j)
		{
			for (int i = 0; i < region.width(); ++i)
			{
				if (*par)
					hist[j]++;

				++par;
			}
			par += w;
		}
	}
	////////////////////////////////////////////////////////////////////////////

	// Вычисление смешанного центрального момента второго порядка
	template<class SOME_RECT, class MASK_TYPE>
    double calc_s_mu(const SOME_RECT& region, double center_mass_r, double center_mass_c, double area, uint32_t frame_width, const MASK_TYPE* mask)
	{
		double sum2(0.);
		int w = frame_width - region.width();
		const MASK_TYPE *par = mask + region.get_left() + frame_width * region.get_top();

		for (int j = 0; j < region.height(); ++j)
		{
			for (int i = 0; i < region.width(); ++i)
			{
				if (*par)
					sum2 += (i - center_mass_r) * (j - center_mass_c);

				++par;
			}
			par += w;
		}
		return sum2 /= area;
	}
	////////////////////////////////////////////////////////////////////////////

	// Построение гистограммы яркости
	template<class MASK_TYPE>
    void calculate_hist(const uchar* buf, int pitch, int pixel_size, int c_x, int c_y, int width, int height, hist_cont& hist, uint32_t frame_width, const MASK_TYPE* mask)
	{
		// image buffer
        buf += pixel_size * c_x + pitch * c_y;
        int w1 = pitch - pixel_size * width;

		// mask buffer
		int padding_par = frame_width - width;
		const MASK_TYPE *par = mask + c_x + frame_width * c_y;

		int total_sum = 0;

		// calculate hist for all masked pixel in region
		for (int j = 0; j < height; ++j)
		{
			for (int i = 0; i < width; ++i)
			{
				if (*par)
				{
                    if (pixel_size == 1)
						hist[*buf]++;
					else
						hist[RGB_2_Y<uchar>(buf)]++;

					++total_sum;
				}
                buf += pixel_size;
				++par;
			}
			par += padding_par;
			buf += w1;
		}

        // нормализуем
		const hist_cont::value_type norm_factor = static_cast<hist_cont::value_type>(total_sum);
		std::for_each(hist.begin(), hist.end(), [norm_factor](hist_cont::value_type& v) { v /= norm_factor; });
	}
	//////////////////////////////////////////////////////////////////

	double bhattacharrya_dist(const hist_cont& source, const hist_cont& dest); // Расстояние между гистограмами

	double calc_center_mass(const hist_cont& arr, double& area);               // Вычисление центра масс по гистограмме	
	double calc_mu(const hist_cont& arr, double center_mass, double area);     // Вычисление центрального момента второго порядка
	////////////////////////////////////////////////////////////////////////////

	// Получение значения контраста изображения
    float_t get_contrast_rgb(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, uint32_t pixel_size);
	////////////////////////////////////////////////////////////////////////

	// Копирование RGB24-буфера на RGB32-буфер
    void copy_24to32(uchar* dest_buf, uint32_t dest_pitch, const uchar* src_buf, uint32_t src_width, uint32_t src_heght);
	
	////////////////////////////////////////////////////////////////////////
	
	// Копирование RGB24-буфера на RGB24-буфер c переворачиванием
    void copy_24to24_flip(uchar* dest_buf, const uchar* src_buf, uint32_t src_width, uint32_t src_heght);
	////////////////////////////////////////////////////////////////////////
	
	// Копирование RGB32-буфера на RGB24-буфер
    void copy_32to24(uchar* dest_buf, const uchar* src_buf, uint32_t src_heght, uint32_t src_pitch);
	
	// Копирование RGB32-буфера на RGB24-буфер c переворачиванием
    void copy_32to24_flip(uchar* dest_buf, const uchar* src_buf, uint32_t src_heght, uint32_t src_pitch);
	////////////////////////////////////////////////////////////////////////
	
	// Копирование gray-буфера на float-буфер
    void copy_gray_to_float(float *dest_buf, const uchar* src_buf, uint32_t src_width, uint32_t src_heght);
	////////////////////////////////////////////////////////////////////////
	
	// Копирование буфера с resize'ом (метод Пешкова-Брезенхама)
    template <size_t SRC_pixel_size, size_t DEST_pixel_size>
    void StretchLine(uchar* src_buf, int src_width, uchar* dest_buf, int dest_width)
	{
		for (int i = 0; i < dest_width; ++i)
		{
			int c_pixel = (src_width * i) / dest_width;
			if (c_pixel >= src_width)
				c_pixel = src_width - 1;

            uchar* r_src = src_buf + c_pixel * SRC_pixel_size;
            uchar* r_dst = dest_buf + i * DEST_pixel_size;

            for (size_t j = 0, stop_j = std::min(SRC_pixel_size, DEST_pixel_size); j < stop_j; ++j)
			{
				r_dst[j] = r_src[j];
			}
		}
	}
	////////////////////////////////////////////////////////////////////////
	
    template <size_t SRC_pixel_size, size_t DEST_pixel_size>
    void StretchBlt(uchar* src_buf, int src_width, int src_height, uchar* dest_buf, int dest_width, int dest_height, int dest_pitch)
	{
		for (int i = 0; i < dest_height; ++i)
		{
			int c_line = (src_height * i) / dest_height;
			if (c_line  >= src_height)
				c_line = src_height - 1;

            StretchLine<SRC_pixel_size, DEST_pixel_size>(src_buf + SRC_pixel_size * src_width * c_line, src_width, dest_buf + i * dest_pitch, dest_width);
		}
	}
	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////
	
	// Копирование части изображения в отдельный буфер
    template<int pixel_size>
	void copy_buf_from_image(uchar* dest_buf, int dest_left, int dest_right, int dest_top, int dest_bottom, const uchar* src_buf, int src_pitch)
	{
        src_buf += pixel_size * dest_left + dest_top * src_pitch;
        int pitch_dest = pixel_size * (dest_right - dest_left);
		for (int j = dest_top; j < dest_bottom; ++j)
		{
			memcpy(dest_buf, src_buf, pitch_dest);
			dest_buf += pitch_dest;
			src_buf += src_pitch;
		}
	}
	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	
	template<class RET_TYPE, size_t SIZE>
	RET_TYPE pixel_metric(const uchar* p1, const uchar* p2)
	{
		RET_TYPE ret_val(0);
		for (size_t i = 0; i < SIZE; ++i)
		{
			ret_val += abs((int)p1[i] - (int)p2[i]);
		}
		return ret_val;
	}
	////////////////////////////////////////////////////////////////////////////

    template<uchar R, uchar G, uchar B, int pixel_size> inline
    void paint_point(uchar* buf, uint32_t pitch, int x, int y)                  // Рисование на RGB24 кадре точки
	{
        buf += pixel_size * x + y * pitch;

		buf[0] = B;
		buf[1] = G;
		buf[2] = R;
	}
	////////////////////////////////////////////////////////////////////////
	
    template<uchar R, uchar G, uchar B, int pixel_size>
    void paint_h_line(uchar* buf, uint32_t pitch, int x1, int x2, int y)        // Рисование на RGB буфере горизонтальной,
	{
        buf += pixel_size * x1 + y * pitch;
        for (; x1 < x2; ++x1, buf += pixel_size)
		{
			buf[0] = B;
			buf[1] = G;
			buf[2] = R;
		}
	}
	////////////////////////////////////////////////////////////////////////
	
    template<uchar R, uchar G, uchar B, int pixel_size>
    void paint_v_line(uchar* buf, uint32_t pitch, int x, int y1, int y2)        // вертикальной и
	{
        buf += pixel_size * x + y1 * pitch;
		for (; y1 < y2; ++y1, buf += pitch)
		{
			buf[0] = B;
			buf[1] = G;
			buf[2] = R;
		}
	}
	////////////////////////////////////////////////////////////////////////
	
    template<uchar R, uchar G, uchar B, int pixel_size>
    void paint_line(uchar* buf, uint32_t pitch, int x1, int x2, int y1, int y2) // произвольной линии
	{
		int dx = abs(x2 - x1);
		int dy = abs(y2 - y1);

		int sx = (x2 >= x1) ? 1: -1;
		int sy = (y2 >= y1) ? 1: -1;

		if (dy <= dx)
		{
			int d1 = 2 * dy;
			int d2 = 2 * (dy - dx);
			int d = d1 - dx;
            paint_point<R, G, B, pixel_size>(buf, pitch, x1, y1);

			for (int x = x1 + sx, y = y1, i = 1; i <= dx; ++i, x += sx)
			{
				if (d > 0)
				{
					d += d2;
					y += sy;
				}
				else
					d += d1;
                paint_point<R, G, B, pixel_size>(buf, pitch, x, y);
			}
		}
		else
		{
			int d1 = 2 * dx;
			int d2 = 2 * (dx - dy);
			int d = d1 - dy;
            paint_point<R, G, B, pixel_size>(buf, pitch, x1, y1);

			for (int x = x1, y = y1 + sy, i = 1; i <= dy; ++i, y += sy)
			{
				if (d > 0)
				{
					d += d2;
					x += sx;
				}
				else
					d += d1;
                paint_point<R, G, B, pixel_size>(buf, pitch, x, y);
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
