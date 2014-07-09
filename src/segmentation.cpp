#include "segmentation.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////

	CSegmentation::CSegmentation()
	{
		eq_regions.reserve(2500);
		tmp_regions.reserve(2000);

		frame_width = 0;
		frame_height = 0;
	}
	////////////////////////////////////////////////////////////////////////////

	CSegmentation::~CSegmentation()
	{
	}
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::set_show_objects(bool show_objects)
	{
		if (!show_objects)
			pixels_l.clear();
	}
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::init(uint frame_width_, uint frame_height_, bool& use_cuda_)
	{
		frame_width = frame_width_;
		frame_height = frame_height_;

		const size_t pixels_count = frame_width * frame_height;
		pixels_l.clear();
		pixels_l.resize(pixels_count);

		if (use_cuda_)
        {
#ifdef USE_CUDA
			bool success_malloc = true;
			success_malloc &= d_mask.malloc(pixels_count);
			success_malloc &= d_mask_temp.malloc(pixels_count);

			size_t el_count = pixels_count / (SEGM_BLOCK_SIZE * SEGM_BLOCK_SIZE);
			success_malloc &= d_reg.malloc(el_count);
			success_malloc &= h_reg.malloc(el_count);

			if (!success_malloc)
				use_cuda_ = false;
#else
        assert(false);
#endif
		}
	}
	////////////////////////////////////////////////////////////////////////////

	mask_cont& CSegmentation::get_mask()
    {
		return pixels_l;
	}
	////////////////////////////////////////////////////////////////////////////

#ifdef USE_CUDA
	CCudaBuf<mask_type, true>& CSegmentation::get_device_mask()
	{
		return d_mask;
	}
#endif
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::add_to_region(regions_container& regions, int x, int y)
	{
		// Указатель на регион, уже содержащий добавляемую точку
		regions_container::iterator iter_tmp(regions.end());

		// Ищем существующий регион, подходящий для добавления точки
		for (regions_container::iterator iter = regions.begin(); iter != regions.end();)
		{
			// Нашли такой регион
			if (iter->in_region(x, y))
			{
				iter->add_point(x, y);

				// Если этот регион первый найденный, то запоминаем указатель на него
				if (iter_tmp == regions.end())
				{
					iter_tmp = iter;
					++iter;
					continue;
				}
				// Иначе расширяем первый найденный регион по размерам последующих, а последующие удаляем
				else
				{
					iter_tmp->regions_merging(*iter);
					iter = regions.erase(iter);
					continue;
				}
			}
			++iter;
		}

		// Регион не найден - создаём новый
		if (iter_tmp == regions.end())
			regions.push_back(CObjectRegion(x, x, y, y));
	}
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::morphology_open(bool use_cuda)
	{
		if (use_cuda)
        {
#ifdef USE_CUDA
			// Нет смысла производить морфолоию, она происходит на уровне сегментации
			//morphology(d_mask.buf, d_mask_temp.buf, frame_width, frame_height, (unsigned int)pixels_l.size());
#else
        assert(false);
#endif
		}
		else
		{
			if (morphology_buf.size() != pixels_l.size())
				morphology_buf.resize(pixels_l.size());

			// Структурный элемент - прямоугольник 3х3

			// Эрозия
			mask_type *pl1 = &pixels_l[0] + frame_width;
			mask_type *pl2 = &morphology_buf[0] + frame_width;
			std::fill(&morphology_buf[0], pl2, 0);

			for (uint y = 1; y < frame_height - 1; ++y, ++pl1, ++pl2)
			{
				*pl2 = 0;
				++pl2;
				++pl1;
				for (uint x = 1; x < frame_width - 1; ++x, ++pl1, ++pl2)
				{
					if (*(pl1 - frame_width - 1) && *(pl1 - frame_width) && *(pl1 - frame_width + 1) &&
						*(pl1 - 1) && *pl1 && *(pl1 + 1) &&
						*(pl1 + frame_width - 1) && *(pl1 + frame_width) && *(pl1 + frame_width + 1))
						*pl2 = 1;
					else
						*pl2 = 0;
				}
				*pl2 = 0;
			}
			std::fill(pl2, pl2 + frame_width, 0);

			// Наращивание
			pl1 = &pixels_l[0] + frame_width;
			pl2 = &morphology_buf[0] + frame_width;
			for (uint y = 1; y < frame_height - 1; ++y, ++pl1, ++pl2)
			{
				++pl2;
				++pl1;
				for (uint x = 1; x < frame_width - 1; ++x, ++pl1, ++pl2)
				{
					if (*pl2)
					{
						*(pl1 - frame_width - 1) = 1;
						*(pl1 - frame_width) = 1;
						*(pl1 - frame_width + 1) = 1;
						*(pl1 - 1) = 1;
						*pl1 = 1;
						*(pl1 + 1) = 1;
						*(pl1 + frame_width - 1) = 1;
						*(pl1 + frame_width) = 1;
						*(pl1 + frame_width + 1) = 1;
					}
					else
					{
						*pl1 = 0;
					}
				}
			}
		}
	}
    ////////////////////////////////////////////////////////////////////////////
#ifdef USE_CUDA
    void CSegmentation::copy_gpu2cpu()
    {
		// Возвращаем маску
		cudaMemcpy(&pixels_l[0], d_mask.buf, d_mask.buf_size, cudaMemcpyDeviceToHost);
	}
#endif
	////////////////////////////////////////////////////////////////////////////

#ifdef ADV_OUT
	void CSegmentation::draw_mask(bool use_cuda, uchar* adv_buf_rgb24)
	{
		if (use_cuda)
        {
#ifdef USE_CUDA
			// Возвращаем маску
			cudaMemcpy(&pixels_l[0], d_mask.buf, d_mask.buf_size, cudaMemcpyDeviceToHost);
#else
        assert(false);
#endif
		}

		mask_type *pl = &pixels_l[0];
		for (uint y = 0; y < frame_height; ++y)
		{
			for (uint x = 0; x < frame_width; ++x)
			{
				if (*pl)
				{
					adv_buf_rgb24[0] = 255;
					adv_buf_rgb24[1] = 255;
					adv_buf_rgb24[2] = 255;
				}
				else
				{
					adv_buf_rgb24[0] = 0;
					adv_buf_rgb24[1] = 0;
					adv_buf_rgb24[2] = 0;
				}
				adv_buf_rgb24 += 3;
				++pl;
			}
		}
	}
#endif
	////////////////////////////////////////////////////////////////////////////
#ifdef USE_CUDA
	void CSegmentation::cuda_segmentation(regions_container& regions)
	{
		// Предварительная блочная сегментация
		unsigned int el_count = (frame_width * frame_height) / (SEGM_BLOCK_SIZE * SEGM_BLOCK_SIZE);
		unsigned int reg_bytes = el_count * sizeof(reg_label);
		segmentation(d_mask.buf, d_reg.buf, frame_width, frame_height);
#if 1
		cudaMemcpy(h_reg.buf, d_reg.buf, reg_bytes, cudaMemcpyDeviceToHost);
#else
		cudaMemcpyAsync(h_reg.buf, d_reg.buf, reg_bytes, cudaMemcpyDeviceToHost, 0);
		cudaThreadSynchronize();
#endif

		int max_right = (int)frame_width - 1;
		int max_bottom = (int)frame_height - 1;

		// Окончательная сегментация
		reg_label *pl = h_reg.buf;
		for (uint y = 0, stop_y = frame_height / SEGM_BLOCK_SIZE; y < stop_y; ++y)
		{
			for (uint x = 0, stop_x = frame_width / SEGM_BLOCK_SIZE; x < stop_x; ++x, ++pl)
			{
				// Добавляем точку объекта к регионам выделения
				//if (*pl > (SEGM_BLOCK_SIZE * SEGM_BLOCK_SIZE) / 2)
				if (*pl > SEGM_BLOCK_SIZE / 2)
				{
					int px = x * SEGM_BLOCK_SIZE;
					int py = y * SEGM_BLOCK_SIZE;

					// Указатель на регион, уже содержащий добавляемую точку
					regions_container::iterator iter_tmp(regions.end());

					// Ищем существующий регион, подходящий для добавления точки
					for (regions_container::iterator iter = regions.begin(); iter != regions.end();)
					{
						// Нашли такой регион
						if (iter->near_region(px, py))
						{
							iter->add_rect(std::min(px + SEGM_BLOCK_SIZE, max_right), std::min(py + SEGM_BLOCK_SIZE, max_bottom), *pl);

							// Если этот регион первый найденный, то запоминаем указатель на него
							if (iter_tmp == regions.end())
							{
								iter_tmp = iter;
								++iter;
								continue;
							}
							// Иначе расширяем первый найденный регион по размерам последующих, а последующие удаляем
							else
							{
								iter_tmp->regions_merging(*iter);
								iter = regions.erase(iter);
								continue;
							}
						}
						++iter;
					}

					// Регион не найден - создаём новый
					if (iter_tmp == regions.end())
					{
						int rx = (px + SEGM_BLOCK_SIZE > max_right)? max_right: (px + SEGM_BLOCK_SIZE);
						int ry = (py + SEGM_BLOCK_SIZE > max_bottom)? max_bottom: (py + SEGM_BLOCK_SIZE);
						regions.push_back(CObjectRegion(px, rx, py, ry, *pl));
					}
				}
			}
		}
	}
#endif
	////////////////////////////////////////////////////////////////////////////

	bool CSegmentation::pairs_comp(const eq_regions_cont::value_type &p1, const eq_regions_cont::value_type &p2)
	{
		if (p1.first > p2.first)
			return true;
		else if (p1.first < p2.first)
			return false;
		else
			return p1.second > p2.second;
	}
	//////////////////////////////////////////////////////////////////////////

	void CSegmentation::iterative_segmentation(regions_container& regions)
	{
		// Временный список регионов без учёта эквивалентности
		tmp_regions.clear();
		eq_regions.clear();

		// Первоначальная разметка областей
		mask_type label = 0;
		mask_cont labs;
		labs.reserve(4);
		mask_type *pl = &pixels_l[0];
		for (uint y = 0; y < frame_height; ++y)
		{
			for (uint x = 0; x < frame_width; ++x, ++pl)
			{
				if (*pl)
				{
					labs.clear();

					if (x > 0)
					{
						if (*(pl - 1))
							labs.push_back(*(pl - 1));
					}
					if ((x > 0) && (y > 0))
					{
						if (*(pl - frame_width - 1))
							labs.push_back(*(pl - frame_width - 1));
					}
					if (y > 0)
					{
						if (*(pl - frame_width))
							labs.push_back(*(pl - frame_width));
					}
					if ((y > 0) && (x < frame_width - 1))
					{
						if (*(pl - frame_width + 1))
							labs.push_back(*(pl - frame_width + 1));
					}

					if (labs.empty())
					{
						*pl = ++label;
						tmp_regions.push_back(tmp_regions_cont::value_type(1, CObjectRegion(x, x, y, y)));
					}
					else
					{
						std::sort(labs.begin(), labs.end());
						*pl = labs[0];
						tmp_regions[*pl - 1].second.add_point(x, y);
						for (size_t li = 1; li < labs.size(); ++li)
						{
							if (*pl != labs[li])
							{
								eq_regions_cont::const_iterator it_eq = eq_regions.begin();
								for (; it_eq != eq_regions.end(); ++it_eq)
								{
									if ((it_eq->first == *pl) && (it_eq->second == labs[li]))
										break;
								}
								if (it_eq == eq_regions.end())
									eq_regions.push_back(eq_regions_cont::value_type(*pl, labs[li]));
							}
						}
					}
				}
			}
		}

		std::sort(eq_regions.begin(), eq_regions.end(), pairs_comp);

		// Окончательное добавление регионов с учётом их возможной эквивалентности
		for (size_t i = 0; i < tmp_regions.size(); ++i)
		{
			if (tmp_regions[i].first)
			{
				for (size_t j = 0; j < eq_regions.size();)
				{
					if ((eq_regions[j].first - 1 == (mask_type)i) && tmp_regions[eq_regions[j].second - 1].first)
					{
						tmp_regions[i].second.regions_merging(tmp_regions[eq_regions[j].second - 1].second);
						tmp_regions[eq_regions[j].second - 1].first = 0;
						eq_regions.erase(eq_regions.begin() + j);
					}
					else
					{
						if ((eq_regions[j].second - 1 == (mask_type)i) && tmp_regions[eq_regions[j].first - 1].first)
						{
							tmp_regions[i].second.regions_merging(tmp_regions[eq_regions[j].first - 1].second);
							tmp_regions[eq_regions[j].first - 1].first = 0;
							eq_regions.erase(eq_regions.begin() + j);
						}
						else
						{
							++j;
						}
					}
				}
				regions.push_back(tmp_regions[i].second);
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::recursive_segmentation(regions_container& regions)
	{
		// Рекурсивная сегментация объектов переднего плана на основе восьмисвязности
		mask_type label = 0;
		for (uint y = 0, oi = 0; y < frame_height; ++y)
		{
			for (uint x = 0; x < frame_width; ++x, ++oi)
			{
				if (pixels_l[oi])
				{
					CObjectRegion reg(x, x, y, y);
					++label;
					search_components(label, x, y, reg);
					regions.push_back(reg);
				}
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::search_components(mask_type label, uint x, uint y, CObjectRegion& reg)
	{
		reg.add_point(x, y);

		uint x_ = x - 1;
		uint y_ = y - 1;
		if (y > 0)
		{
			// левая-верхняя
			if ((x > 0) && pixels_l[x_ + y_ * frame_width])
				search_components(label, x_, y_, reg);
			// центральная-верхняя
			x_ = x;
			if (pixels_l[x_ + y_ * frame_width])
				search_components(label, x_, y_, reg);
			//правая-верхняя
			x_ = x + 1;
			if ((x < frame_width - 1) && pixels_l[x_ + y_ * frame_width])
				search_components(label, x, y_, reg);
		}

		y_ = y;
		// левая-центральная
		x_ = x - 1;
		if ((x > 0) && pixels_l[x_ + y_ * frame_width])
			search_components(label, x_, y_, reg);
		// центральная-центральная
		x_ = x;
		pixels_l[x_ + y_ * frame_width] = label;
		// правая-центральная
		x_ = x + 1;
		if ((x < frame_width - 1) && pixels_l[x_ + y_ * frame_width])
			search_components(label, x_, y_, reg);

		if (y < frame_height - 1)
		{
			y_ = y + 1;
			// левая-нижняя
			x_ = x - 1;
			if ((x > 0) && pixels_l[x_ + y_ * frame_width])
				search_components(label, x_, y_, reg);
			// центральная-нижняя
			x_ = x;
			if (pixels_l[x_ + y_ * frame_width])
				search_components(label, x_, y_, reg);
			// правая-нижняя
			x_ = x + 1;
			if ((x < frame_width - 1) && pixels_l[x_ + y_ * frame_width])
				search_components(label, x_, y_, reg);
		}
	}
	////////////////////////////////////////////////////////////////////////////

	void CSegmentation::square_segmentation(regions_container& regions)
	{
		mask_type *pl = &pixels_l[0];
		for (uint y = 0; y < frame_height; ++y)
		{
			for (uint x = 0; x < frame_width; ++x, ++pl)
			{
				// Добавляем точку объекта к регионам выделения
				if (*pl)
					add_to_region(regions, x, y);
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
