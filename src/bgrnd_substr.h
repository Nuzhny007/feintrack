#pragma once

#include "feintrack_params.h"
#include "utils.h"
#include "feintrack_objects.h"
#include "segmentation.h"
#include "some_types.h"
////////////////////////////////////////////////////////////////////////////

namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////

	typedef float ft_float_t;                 // Тип статистических характеристик модели заднего плана
	////////////////////////////////////////////////////////////////////////////

	// Базовый класс для алгоритмов вычитания фона
	class CBackSubstraction
	{
	public:
		CBackSubstraction();
		virtual ~CBackSubstraction();

		virtual bool init(uint width, uint height, color_type buf_type, bool& use_cuda_); // Возвращает true, если инициализация была проведена, иначе - false

		// Вычитание фона
#ifndef ADV_OUT
#ifdef USE_CUDA
		virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask) = 0;
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l) = 0;
#endif
#else
#ifdef USE_CUDA
		virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24) = 0;
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24) = 0;
#endif
#endif

		// Обновляет статистику в регионе
		virtual void update_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region) = 0;

		// Делает значения выборочного среднего в регионе равными текущим значениям пикселей
		virtual void reset_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region) = 0;

		bool get_detect_patches_of_sunlight() const;                           // Используется ли детектор бликов
		void set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_); // Включение/выключение детектора бликов

		virtual void set_sensitivity(int sens_level);         // Задаёт уровень чувствительности для вычитания фона (от 1 до 100)
		virtual int get_sensitivity() const;                  // Получение уровня чувствительности для вычитания фона (от 1 до 100)

		virtual void set_fps(int new_fps);                    // Задание fps

		virtual void enable_back_update(bool enable_val);     // Разрешение/запрещение обновления фона

		virtual void set_show_objects(bool show_objects);     // Показывать/не показывать объекты

		virtual void set_use_cuda(bool use_cuda_);            // Задание использования CUDA

	protected:
		// Параметры обучения низкочастотного фильтра рекурсивного сглаживания
		static const ft_float_t alpha1;               // Для выборочного среднего
		static const ft_float_t alpha2;               // Для среднеквадратичного отклонения

		static const ft_float_t min_sigma_val;        // Минимальное и
		static const ft_float_t max_sigma_val;        // максимальное значение для среднеквадратичного отклонения (используется при вычитании фона)

		uint frame_width;                             // Ширина
		uint frame_height;                            // и высота одного кадра в пикселях
		color_type curr_color_type;                   // Текущий тип цветового пространства анализируемого кадра

		ft_float_t epsilon;                           // Порог, по которому определяется принадлежность пикселя переднему или заднему плану (расстояние Махаланобиса)

		int PIXEL_SIZE;                               // Размер одного пикселя в байтах

		bool use_cuda;                                // Использовать CUDA для вычитания фона и морфологии

		bool init_filter;                             // Собрана ли первоначальная статистика

		int fps;                                      // Количество кадров в секунду, поступающих на фильтр. Обновление статистики производится раз в секунду. Время будет измеряться по кадрам.

		bool need_background_update;                  // Обновлять ли модель заднего плана

		bool is_patch_of_sunlight(const ft_float_t* float_src); // Является ли данный пиксель частью блика

	private:
		static const ft_float_t min_sens;             // Минимальное и
		static const ft_float_t max_sens;             // максимальное значение порога при вычитании фона (epsilon)

		bool detect_patches_of_sunlight;              // Использовать детектор бликов
		static const ft_float_t sunlight_threshold;   // Порог значение пикселя для определения блика
	};
	////////////////////////////////////////////////////////////////////////////

	// Параметры нормального распределения
	struct CNormParams
	{
		CNormParams()
			: mu(0), sigma(0)
		{
		}
		ft_float_t mu;       // Выборочное среднее
		ft_float_t sigma;    // Среднеквадратичное отклонение

		// Пересчитывает значение выборочного среднего с помощью экспоненциального сглаживания
		template<class VAL_TYPE>
		void recalc_mu(VAL_TYPE new_val, ft_float_t alpha)
		{
			mu = (1 - alpha) * mu + alpha * new_val;
		}
		// Пересчитывает значение среднеквадратичного отклонения с помощью экспоненциального сглаживания
		template<class VAL_TYPE>
		void recalc_sigma(VAL_TYPE new_val, ft_float_t alpha)
		{
			sigma = sqrt((1 - alpha) * sqr(sigma) + alpha * sqr(new_val - mu));
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// Параметры нормального распределения для пикселя (на каждый компонент цвета своя случайная величина)
	template<size_t NORM_COUNT>
	struct CNormParamsPixel
	{
		CNormParams p[NORM_COUNT];  // Параметры нормального распределения для каждого цвета пикселя

		// Пересчитывает значение выборочного среднего для всех компонент
		template<class VAL_TYPE>
		void recalc_mu(VAL_TYPE* new_val, ft_float_t alpha)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].recalc_mu(new_val[i], alpha);
			}
		}

		// Пересчитывает значение среднеквадратичного отклонения для всех компонент
		template<class VAL_TYPE>
		void recalc_sigma(VAL_TYPE* new_val, ft_float_t alpha, ft_float_t min_sigma_val, ft_float_t max_sigma_val)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].recalc_sigma(new_val[i], alpha);
				set_range(p[i].sigma, min_sigma_val, max_sigma_val);
			}
		}

		// Создание статистической модели заднего плана на очередном кадре
		template<class VAL_TYPE>
		void create_statistic(VAL_TYPE* new_val, ft_float_t curr_frame)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].mu += new_val[i];
				p[i].sigma += sqr(new_val[i] - p[i].mu / curr_frame);
			}
		}
		// Завершение создания модели заднего плана
		template<class VAL_TYPE>
		void end_create_statistic(VAL_TYPE* new_val, ft_float_t curr_frame, ft_float_t min_sigma_val, ft_float_t max_sigma_val)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].mu = (p[i].mu + new_val[i]) / curr_frame;

				p[i].sigma += sqr(new_val[i] - p[i].mu / curr_frame);
				p[i].sigma = sqrt(p[i].sigma / (curr_frame - 1));
				set_range(p[i].sigma, min_sigma_val, max_sigma_val);
			}
		}

		// Проверка на принадлежность пикселя заднему плану
		bool is_back(ft_float_t *new_val, ft_float_t *eps) const
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				if (eps[i] * p[i].sigma < abs(p[i].mu - new_val[i]))
					return false;
			}
			return true;
		}

		// Задание значений модели
		template<class VAL_TYPE>
		void set_mu_sigma(VAL_TYPE* new_val, ft_float_t new_sigma)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].mu = new_val[i];
				p[i].sigma = new_sigma;
			}
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// Вычитание фона на основе моделирования каждого пикселя случайным нормально распределённым вектором
	class CNormBackSubstraction: public CBackSubstraction
	{
	public:
		CNormBackSubstraction();
		~CNormBackSubstraction();

		bool init(uint width, uint height, color_type buf_type, bool& use_cuda_); // Возвращает true, если инициализация была проведена, иначе - false

		// Вычитание фона
#ifndef ADV_OUT
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l);
#endif
#else
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24);
#endif
#endif

		void update_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region); // Обновляет статистику в регионе
		void reset_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region);  // Делает значения выборочного среднего в регионе равными текущим значениям пикселей

		void set_fps(int new_fps);                    // Задание fps
		void set_show_objects(bool show_objects);     // Показывать/не показывать объекты

	private:
		typedef CNormParamsPixel<3> rgb_param_type;            //
		typedef CNormParamsPixel<1> gray_param_type;           //
		typedef std::vector<rgb_param_type> rgb_params_cont;   // 
		typedef std::vector<gray_param_type> gray_params_cont; // 

		int contrast_time;                            // Время для проверки контраста кадра
		int contrast_frame;                           // Счётчик кадров для отсчёта времени до contrast_time
		static const float_t contrast_threshold;      // Порог значения контраста, ниже которого отключается детектор теней

		bool use_shadow_detector;                     // Использовать детектор теней

		rgb_params_cont rgb_params;                   // Массив размером frame_width * frame_height, в котором хранятся статистические данные о каждом пикселе
		gray_params_cont gray_params;                 // Массив размером frame_width * frame_height, в котором хранятся статистические данные о каждом пикселе

#ifdef USE_CUDA
        CCudaBuf<BGRXf, false> h_frame_bgrxf;         // Буфер кадра для копирования в видеопамять
		CCudaBuf<long, true> d_bgr32;                 // Видеопамять под кадр
		CCudaBuf<float, true> d_params_b_mu;          // Видеопамять под параметры модели заднего плана
		CCudaBuf<float, true> d_params_b_sigma;       // Видеопамять под параметры модели заднего плана
		CCudaBuf<float, true> d_params_g_mu;          // Видеопамять под параметры модели заднего плана
		CCudaBuf<float, true> d_params_g_sigma;       // Видеопамять под параметры модели заднего плана
		CCudaBuf<float, true> d_params_r_mu;          // Видеопамять под параметры модели заднего плана
		CCudaBuf<float, true> d_params_r_sigma;       // Видеопамять под параметры модели заднего плана
#endif

		// Вычитание фона
		template<class PARAMS_CONT>
#ifndef ADV_OUT
#ifdef USE_CUDA
		int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params);
#endif
#else
#ifdef USE_CUDA
		int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#endif
#endif

		// Обновляет статистику в регионе
		template<class PARAMS_CONT>
		void update_statistic_in_region(const uchar* buf, uint pitch, PARAMS_CONT& params, const CObjectRegion& region);
		
		// Делает значения выборочного среднего в регионе равными текущим значениям пикселей
		template<class PARAMS_CONT>
		void reset_statistic_in_region(const uchar* buf, uint pitch, PARAMS_CONT& params, const CObjectRegion& region);
	};
	////////////////////////////////////////////////////////////////////////////

	// Параметры нормального распределения + вес параметров как части процесса
	template<size_t NORM_COUNT>
	struct CNormWeightProcess: public CNormParamsPixel<NORM_COUNT>
	{
		CNormWeightProcess()
			: weight(0)
		{
		}

		ft_float_t weight;       // Вес процесса
	};
	////////////////////////////////////////////////////////////////////////////

	// Список процессов, соответствующий каждому пикселю
	template<size_t NORM_COUNT>
	struct CNormWeightParams3
	{
		CNormWeightParams3()
			: curr_proc(0), created_processes(1)
		{
		}

		static const size_t PROC_PER_PIXEL = 3;                   // Число процессов на каждый пиксель
		CNormWeightProcess<NORM_COUNT> proc_list[PROC_PER_PIXEL]; // Параметры распределения на каждый процесс
		size_t curr_proc;                                         // Текущий процесс
		size_t created_processes;                                 // Количество уже созданных процессов

		// Создание статистической модели заднего плана на очередном кадре
		template<class VAL_TYPE>
		void create_statistic(VAL_TYPE* new_val, ft_float_t curr_frame)
		{
			for (size_t proc_ind = 0; proc_ind < PROC_PER_PIXEL; ++proc_ind)
			{
				proc_list[proc_ind].create_statistic(new_val, curr_frame);
			}
		}
		// Завершение создания модели заднего плана
		template<class VAL_TYPE>
		void end_create_statistic(VAL_TYPE* new_val, ft_float_t curr_frame, ft_float_t min_sigma_val, ft_float_t max_sigma_val)
		{
			for (size_t proc_ind = 0; proc_ind < PROC_PER_PIXEL; ++proc_ind)
			{
				proc_list[proc_ind].end_create_statistic(new_val, curr_frame, min_sigma_val, max_sigma_val);
			}
			proc_list[curr_proc].weight = 1;
		}

		// Проверка на принадлежность пикселя заднему плану
		bool is_back(ft_float_t *new_val, ft_float_t *eps, ft_float_t alpha1, ft_float_t alpha2, ft_float_t alpha3, ft_float_t min_sigma_val, ft_float_t max_sigma_val, ft_float_t weight_threshold)
		{
			bool find_process = false;

			for (size_t proc_ind = 0; proc_ind < created_processes; ++proc_ind)
			{
				// Ищем процесс, который лучше соответствует текущему значению пикселя
				if (proc_list[proc_ind].is_back(new_val, eps))
				{
					// Процесс найден - уточняем его параметры
					curr_proc = proc_ind;
					
					// Оценки мат. ожидания и дисперсии обновляются с помощью низкочастотного фильтра рекурсивного сглаживания
					proc_list[curr_proc].recalc_mu(new_val, alpha1);
					proc_list[curr_proc].recalc_sigma(new_val, alpha2, min_sigma_val, max_sigma_val);

					find_process = true;
					break;
				}
			}
			if (!find_process) // Процесс не найден
			{
				// Создаём новый процесс или,
				if (created_processes < PROC_PER_PIXEL)
				{
					++created_processes;
					curr_proc = created_processes - 1;

					proc_list[curr_proc].set_mu_sigma(new_val, min_sigma_val);

					find_process = true;
				}
				// если количество процессов равно PROC_PER_PIXEL, ищем процесс с наименьшим весом
				else
				{
					ft_float_t min_weight = proc_list[0].weight;
					size_t min_proc = 0;
					for (size_t proc_ind = 1; proc_ind < created_processes; ++proc_ind)
					{
						if (proc_list[proc_ind].weight < min_weight)
						{
							min_proc = proc_ind;
							min_weight = proc_list[proc_ind].weight;
						}
					}
					curr_proc = min_proc;
					proc_list[curr_proc].set_mu_sigma(new_val, min_sigma_val);
				}
			}

			// Обновление весов процессов
			if (find_process)
			{
				for (size_t proc_ind = 0; proc_ind < created_processes; ++proc_ind)
				{
					proc_list[proc_ind].weight = (1 - alpha3) * proc_list[proc_ind].weight + alpha3 * ((proc_ind == curr_proc) ? 1 : 0);
				}
			}

			return proc_list[curr_proc].weight > weight_threshold;
		}

		// Задание значений модели
		template<class VAL_TYPE>
		void set_mu_sigma(VAL_TYPE* new_val, ft_float_t new_sigma)
		{
			proc_list[curr_proc].set_mu_sigma(new_val, new_sigma);
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// Вычитание фона на основе моделирования каждого пикселя случайным нормально распределённым вектором
	class CGaussianMixtureBackSubstr: public CBackSubstraction
	{
	public:
		CGaussianMixtureBackSubstr();
		~CGaussianMixtureBackSubstr();

		bool init(uint width, uint height, color_type buf_type, bool& use_cuda_); // Возвращает true, если инициализация была проведена, иначе - false

		// Вычитание фона
#ifndef ADV_OUT
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l);
#endif
#else
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24);
#endif
#endif

		void update_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region); // Обновляет статистику в регионе
		void reset_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region);  // Делает значения выборочного среднего в регионе равными текущим значениям пикселей

		void set_show_objects(bool show_objects);              // Показывать/не показывать объекты

	private:
		typedef CNormWeightParams3<3> rgb_param_type;          //
		typedef CNormWeightParams3<1> gray_param_type;         //
		typedef std::vector<rgb_param_type> rgb_params_cont;   // 
		typedef std::vector<gray_param_type> gray_params_cont; // 

		rgb_params_cont rgb_params;                            // Массив размером frame_width * frame_height, в котором хранятся статистические данные о каждом пикселе
		gray_params_cont gray_params;                          // Массив размером frame_width * frame_height, в котором хранятся статистические данные о каждом пикселе

#ifdef USE_CUDA
        CCudaBuf<BGRXf, false> h_frame_bgrxf;                  // Буфер кадра для копирования в видеопамять
		CCudaBuf<long, true> d_bgr32;                          // Видеопамять под кадр
		CCudaBuf<long, true> d_curr_processes;                 // Видеопамять с индексами текущих процессов для каждого пикселя
		CCudaBuf<long, true> d_created_processes;              // Видеопамять с числом созданных процессов для каждого пикселя
		CCudaBuf<BgrndProcess, true> d_process1;               // Видеопамять с параметрами модели фона для первого процесса
		CCudaBuf<BgrndProcess, true> d_process2;               // Видеопамять с параметрами модели фона для второго процесса
		CCudaBuf<BgrndProcess, true> d_process3;               // Видеопамять с параметрами модели фона для третьего процесса
#endif

		static const ft_float_t alpha3;                        // Параметр обучения низкочастотного фильтра рекурсивного сглаживания для веса процесса
		static const ft_float_t weight_threshold;              // Порог для веса процесса

		// Вычитание фона
		template<class PARAMS_CONT>
#ifndef ADV_OUT
#ifdef USE_CUDA
		int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params);
#endif
#else
#ifdef USE_CUDA
		int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#endif
#endif

		// Делает значения выборочного среднего в регионе равными текущим значениям пикселей
		template<class PARAMS_CONT>
		void reset_statistic_in_region(const uchar* buf, uint pitch, PARAMS_CONT& params, const CObjectRegion& region);
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
