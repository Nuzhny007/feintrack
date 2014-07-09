#include "utils.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////
	// Вычисление центра масс по гистограмме	
	double calc_center_mass(const hist_cont& arr, double& area)
	{
		double sum2(0.);
		area = 0.;
		for (size_t i = 0; i < arr.size(); ++i)
		{
			area += arr[i];
			sum2 += i * arr[i];
		}
		return sum2 /= area;
	}
	////////////////////////////////////////////////////////////////////////////

	// Вычисление центрального момента второго порядка
	double calc_mu(const hist_cont& arr, double center_mass, double area)
	{
		double sum2(0.);
		for (size_t i = 0; i < arr.size(); ++i)
		{
			if (arr[i] > 0.5)
				sum2 += arr[i] * sqr(i - center_mass);
		}
		return sum2 /= area;
	}
	////////////////////////////////////////////////////////////////////////////

	float_t get_contrast_rgb24(const uchar* buf, uint pitch, uint width, uint height)
	{
		//Получаем выборку значений яркости пикселей на равных интервалах
		//Возвращаемое значение - отношение максимальной частоты на каком-либо интервале к общему количеству пикселей

        const uint INTERVALS_COUNT = 5;
		uint values[INTERVALS_COUNT] = {0};
		float_t interval_len = RGB_2_Y<int>(0xff, 0xff, 0xff) / (float_t)INTERVALS_COUNT;

		for (uint y = 0; y < height; ++y, buf += pitch - 3 * width)
		{
			for (uint x = 0; x < width; ++x, buf += 3)
			{
				float_t tmp_y = (float_t)RGB_2_Y<int>(buf[2], buf[1], buf[0]);
				for (uint j = 0; j < INTERVALS_COUNT; ++j)
				{
					if ((j + 1 == INTERVALS_COUNT) || (tmp_y < (j + 1) * interval_len))
					{
						values[j]++;
						break;
					}
				}
			}
		}

		uint max_val = values[0];
		for (uint j = 1; j < INTERVALS_COUNT; ++j)
		{
			if (max_val < values[j])
				max_val = values[j];
		}

		return (float_t)max_val / (float_t)(width * height);
	}
	////////////////////////////////////////////////////////////////////////////

	double bhattacharrya_dist(const hist_cont& source, const hist_cont& dest)
	{
		double result = 0;
		for (size_t i = 0, stop_i = source.size(); i < stop_i; ++i)
		{
			result += std::sqrt(source[i] * dest[i]);
		}
		return result;
	}
	////////////////////////////////////////////////////////////////// 

	void copy_24to32(uchar* dest_buf, uint dest_pitch, const uchar* src_buf, uint src_width, uint src_heght)
	{
		const ptrdiff_t src_pitch = 3 * src_width;
        unsigned int* dst = (unsigned int* )dest_buf;
		const ptrdiff_t dst_pitch_delta = (dest_pitch - 4 * src_width) / 4;
		for (size_t i = 0; i < src_heght; ++i)
		{
			for (const uchar* stop = src_buf + src_pitch; src_buf != stop; src_buf += 3)
			{
                *dst = *((unsigned int*)src_buf);
				++dst;
			}
			dst += dst_pitch_delta;
		}
	}
	////////////////////////////////////////////////////////////////////////
	void copy_24to24_flip(uchar* dest_buf, const uchar* src_buf, uint src_width, uint src_heght)
	{
		uint src_pitch = 3 * src_width;
		src_buf += src_pitch * src_heght;
        const uchar* from_ptr(nullptr);
		for (uint i = 0; i < src_heght; ++i)
		{
			from_ptr = src_buf - src_pitch;
			for (; from_ptr != src_buf; from_ptr += 3, dest_buf += 3)
			{
				dest_buf[0] = from_ptr[0];
				dest_buf[1] = from_ptr[1];
				dest_buf[2] = from_ptr[2];
			}
			src_buf -= src_pitch;
		}
	}
	////////////////////////////////////////////////////////////////////////
	void copy_32to24(uchar* dest_buf, const uchar* src_buf, uint src_heght, uint src_pitch)
	{
		const uchar* end_ptr = src_buf + src_pitch * src_heght;
		for (; src_buf != end_ptr; src_buf += 4, dest_buf += 3)
		{
			dest_buf[0] = src_buf[0];
			dest_buf[1] = src_buf[1];
			dest_buf[2] = src_buf[2];
		}
	}
	////////////////////////////////////////////////////////////////////////
	void copy_32to24_flip(uchar* dest_buf, const uchar* src_buf, uint src_heght, uint src_pitch)
	{
		src_buf += src_pitch * src_heght;
        const uchar* from_ptr(nullptr);
		for (uint i = 0; i < src_heght; ++i)
		{
			from_ptr = src_buf - src_pitch;
			for (; from_ptr != src_buf; from_ptr += 4, dest_buf += 3)
			{
				dest_buf[0] = from_ptr[0];
				dest_buf[1] = from_ptr[1];
				dest_buf[2] = from_ptr[2];
			}
			src_buf -= src_pitch;
		}
	}
	////////////////////////////////////////////////////////////////////////
	void copy_gray_to_float(float *dest_buf, const uchar* src_buf, uint src_width, uint src_heght)
	{
		for (size_t i = 0; i < src_heght; ++i)
		{
			for (const uchar* stop = src_buf + src_width; src_buf != stop; ++src_buf)
			{
				*dest_buf = float(*src_buf);
				++dest_buf;
			}
		}
	}
	////////////////////////////////////////////////////////////////////////
    bool is_intersect(float_t x1_1, float_t y1_1, float_t x2_1, float_t y2_1,
		float_t x1_2, float_t y1_2, float_t x2_2, float_t y2_2)
	{
		const float_t eps = 0.00001; //Константа для сравнения на равентсво

		//Получение уравнения прямой, содержащей 1-й отрезок
		float_t a1(0), b1(0); //Коэффициенты 1-й прямой
		bool trivial1(false); //Вырожденный случай - прямая перпендикулярна оси OX

		if (abs(x1_1 - x2_1) < eps)
		{
			trivial1 = true;
		}
		else
		{
			a1 = (y2_1 - y1_1) / (x2_1 - x1_1);
			b1 = (x2_1 * y1_1 - x1_1 * y2_1) / (x2_1 - x1_1);
		}

		//Получение уравнения прямой, содержащей 2-й отрезок
		float_t a2(0), b2(0); //Коэффициенты 2-й прямой
		bool trivial2(false); //Вырожденный случай - прямая перпендикулярна оси OX

		if (abs(x1_2 - x2_2) < eps) //Вырожденный случай - прямая перпендикулярна оси OX
		{
			trivial2 = true;
		}
		else
		{
			a2 = (y2_2 - y1_2) / (x2_2 - x1_2);
			b2 = (x2_2 * y1_2 - x1_2 * y2_2) / (x2_2 - x1_2);
		}

		//Определение координат пересечения прямых
		float_t x(0), y(0);

		if (trivial1)
		{
			if (trivial2)
				return (abs(x1_1 - x1_2) < eps);
			else
				x = x1_1;
			y = a2 * x + b2;
		}
		else
		{
			if (trivial2)
			{
				x = x1_2;
			}
			else
			{
				if (abs(a2 - a1) > eps)
					x = (b1 - b2) / (a2 - a1);
				else
					return false;
			}
			y = a1 * x + b1;
		}

		return in_range(x, std::min(x1_1, x2_1), std::max(x1_1, x2_1) + eps) && in_range(x, std::min(x1_2, x2_2), std::max(x1_2, x2_2) + eps)
			   &&
			   in_range(y, std::min(y1_1, y2_1), std::max(y1_1, y2_1) + eps) && in_range(y, std::min(y1_2, y2_2), std::max(y1_2, y2_2) + eps);
	}
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
