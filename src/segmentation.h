#pragma once

#include "feintrack_params.h"
#include "utils.h"
#include "some_types.h"
#include "feintrack_objects.h"
////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	// Сегментация объектов переднего плана в регионы
	class CSegmentation
	{
	public:
		CSegmentation();
		~CSegmentation();

		void init(uint frame_width_, uint frame_height_, bool& use_cuda_); // Инициализация внутренних структур

#ifdef ADV_OUT
		void draw_mask(bool use_cuda, uchar* adv_buf_rgb24);           // Вывод на дополнительный буфер результатов вычитания фона
#endif

		void morphology_open(bool use_cuda); // Операция математической морфологии "открытие" для результатов вычитания фона

		void square_segmentation(regions_container& regions);                   // Сегментация объектов как прямоугольных регионов
		void iterative_segmentation(regions_container& regions);                // Итеративная сегментация бинарного изображения на основе 8-ми связности
		void recursive_segmentation(regions_container& regions);                // Рекурсивная сегментация бинарного изображения на основе 8-ми связности

#ifdef USE_CUDA
        void cuda_segmentation(regions_container& regions);                     // Сегментация объектов с использованием CUDA
		void copy_gpu2cpu();                          // Копирование маски из видео в системную память
#endif

		void set_show_objects(bool show_objects);     // Показывать/не показывать объекты

        mask_cont& get_mask();                        // Возвращает маску кадра
#ifdef USE_CUDA
		CCudaBuf<mask_type, true>& get_device_mask(); // Получение маски из видеопамяти
#endif

	private:

		typedef std::vector<std::pair<mask_type, mask_type> > eq_regions_cont;
		eq_regions_cont eq_regions;              // Список эквивалентности регионов для итеративной сегментации
		static bool pairs_comp(const eq_regions_cont::value_type &p1, const eq_regions_cont::value_type &p2); // Cравнение эквивалентных регионов
		typedef std::vector<std::pair<mask_type, CObjectRegion> > tmp_regions_cont;
		tmp_regions_cont tmp_regions;            // Временный список регионов для итеративной сегментации
		void search_components(mask_type label, uint x, uint y, CObjectRegion& reg); // Рекурсивный поиск связных компонет при сегментации регионов

		mask_cont pixels_l;                           // Массив размером frame_width * frame_height, в котором хранится результат вычитания фона

#ifdef USE_CUDA
        CCudaBuf<mask_type, true> d_mask;             // Видеопамять под маску
		CCudaBuf<mask_type, true> d_mask_temp;        // Видеопамять для операции математической морфологии
		CCudaBuf<reg_label, true> d_reg;              // Видеопамять для предварительной сегментации

		CCudaBuf<reg_label, false> h_reg;             // Оперативная память для предварительной сегментации
#endif

		mask_cont morphology_buf;                     // Массив размером frame_width * frame_height для хранения промежуточного результата операции математической морфологии "открытие"

		uint frame_width;                             // Ширина
		uint frame_height;                            // и высота одного кадра в пикселях

		void add_to_region(regions_container& regions, int x, int y); // Поиск подходящего региона
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
