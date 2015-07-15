#pragma once

#include "feintrack_dll.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
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

        unsigned int uid;          // Уникальный идентификатор зоны
		cstring name;       // Имя зоны

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
		cstring name; // Имя линии

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

	// Профиль с настройками finetrack'a
	struct CFeintrackProfile
	{
		CFeintrackProfile()
            :
              profile_name("Profile 1"),
              sensitivity(67),
			fps(25),
			show_objects(false),
			use_cuda(false),
			cuda_device_ind(0),
            bgrnd_type(norm_back),
			use_recognition(false),
			use_morphology(true),
			show_left_objects(true),
			show_trajectory(false),
            detect_patches_of_sunlight(false),
            cut_shadows(false),
			selection_time(12),
            min_region_width(5),
            min_region_height(5),
            analyze_area(0, 100, 0, 100),
			left_object_time1_sec(15),
			left_object_time2_sec(1 * 60),
			left_object_time3_sec(2 * 60),
            use_square_segmentation(true)
    {
		}
		~CFeintrackProfile()
		{
		}

		cstring profile_name;            // Имя профиля

		int sensitivity;                 // Чувствительность (для метода вычитания фона)
		int fps;                         // Количество кадров в секунду, подаваемых на feintrack
		bool show_objects;               // Отображать объекты
		bool use_cuda;                   // Использовать CUDA для вычитания фона и морфологии
		int cuda_device_ind;             // Индекс используемого устройства, поддерживающего CUDA

		bgrnd_substr_types bgrnd_type;   // Тип алгоритма вычитания фона

		bool use_recognition;            // Использовать распознавание людей
		bool use_morphology;             // Использовать фильтр качающихся деревьев
		bool show_left_objects;          // Отображать оставленные предметы
		bool show_trajectory;            // Отображать траектории объектов

		bool detect_patches_of_sunlight; // Использовать детектор бликов

		bool cut_shadows;                // Убирать тени, основываясь на анализе формы объекта

		zones_cont zones;                // Список зон
		lines_cont lines;                // Список разделительных линий

		int selection_time;              // Время фильтрации новых объектов

		int min_region_width;            // Минимальная ширина и
		int min_region_height;           // высота региона, который будет детектироваться

		RECT_ analyze_area;              // Часть кадра, которая будет анализироваться

		int left_object_time1_sec;       // Время в секундах, после которого объект считается похожим на оставленный
		int left_object_time2_sec;       // Время в секундах, после которого объект считается оставленным
		int left_object_time3_sec;       // Время в секундах, после которого оставленный предмет удаляется

		bool use_square_segmentation;    // Использовать сегментацию объектов как прямоугольных регионов. Иначе используется 8-ми связность
	};
    typedef std::deque<CFeintrackProfile> profiles_cont;
	////////////////////////////////////////////////////////////////////////////

	// Параметры конфигурации Feintrack'a
	class CFeintrackParams
	{
	public:
		CFeintrackParams()
			: curr_profile_ind(0)
		{
			profiles.push_back(CFeintrackProfile());
		}
		~CFeintrackParams()
		{
		}

		const CFeintrackProfile &get_curr_profile() const // Получение текущего профиля
		{
			if (curr_profile_ind < profiles.size())
				return profiles[curr_profile_ind];
			else
				return profiles[0];
		}
		CFeintrackProfile &get_curr_profile()             // Получение текущего профиля
		{
			if (curr_profile_ind < profiles.size())
				return profiles[curr_profile_ind];
			else
				return profiles[0];
		}

		profiles_cont profiles;            // Профили настроек
		size_t curr_profile_ind;           // Индекс выбранного профиля

		cstring channel_name;              // Имя видеоканала, к которому относятся параметры

		int get_sensitivity() const                                             //
		{
			return get_curr_profile().sensitivity;
		}
		void set_sensitivity(int sensitivity_)                                  //
		{
			get_curr_profile().sensitivity = sensitivity_;
		}

		int get_selection_time() const                                          //
		{
			return get_curr_profile().selection_time;
		}
		void set_selection_time(int selection_time_)                            //
		{
			get_curr_profile().selection_time = selection_time_;
		}

		int get_min_region_width() const                                        //
		{
			return get_curr_profile().min_region_width;
		}
		void set_min_region_width(int min_region_width_)                        //
		{
			get_curr_profile().min_region_width = min_region_width_;
		}
		int get_min_region_height() const                                       //
		{
			return get_curr_profile().min_region_height;
		}
		void set_min_region_height(int min_region_height_)                      //
		{
			get_curr_profile().min_region_height = min_region_height_;
		}

		RECT_ get_analyze_area() const                                          //
		{
			return get_curr_profile().analyze_area;
		}
		void set_analyze_area(const RECT_& analyze_area_)                       //
		{
			get_curr_profile().analyze_area = analyze_area_;
		}

		int get_fps() const                                                     //
		{
			return get_curr_profile().fps;
		}
		void set_fps(int fps_)                                                  //
		{
			get_curr_profile().fps = fps_;
		}

		bool get_show_objects() const                                           //
		{
			return get_curr_profile().show_objects;
		}
		void set_show_objects(bool show_objects_)                               //
		{
			get_curr_profile().show_objects = show_objects_;
		}

		bool get_use_cuda() const                                               //
		{
			return get_curr_profile().use_cuda;
		}
		void set_use_cuda(bool use_cuda_, int cuda_device_ind_)                 //
		{
			get_curr_profile().use_cuda = use_cuda_;
			get_curr_profile().cuda_device_ind = cuda_device_ind_;
		}
		int get_cuda_device() const                                             //
		{
			return get_curr_profile().cuda_device_ind;
		}

		bgrnd_substr_types get_bgrnd_type() const                               //
		{
			return get_curr_profile().bgrnd_type;
		}
		void set_bgrnd_type(bgrnd_substr_types bgrnd_type_)                     //
		{
			get_curr_profile().bgrnd_type = bgrnd_type_;
		}

		bool get_use_recognition() const                                        //
		{
			return get_curr_profile().use_recognition;
		}
		void set_use_recognition(bool use_recognition_)                         //
		{
			get_curr_profile().use_recognition = use_recognition_;
		}

		bool get_use_morphology() const                                         //
		{
			return get_curr_profile().use_morphology;
		}
		void set_use_morphology(bool use_morphology_)                           //
		{
			get_curr_profile().use_morphology = use_morphology_;
		}

		int get_left_object_time1_sec() const                                   //
		{
			return get_curr_profile().left_object_time1_sec;
		}
		void set_left_object_time1_sec(int left_object_time1_sec_)              //
		{
			get_curr_profile().left_object_time1_sec = left_object_time1_sec_;
		}
		int get_left_object_time2_sec() const                                   //
		{
			return get_curr_profile().left_object_time2_sec;
		}
		void set_left_object_time2_sec(int left_object_time2_sec_)              //
		{
			get_curr_profile().left_object_time2_sec = left_object_time2_sec_;
		}
		int get_left_object_time3_sec() const                                   //
		{
			return get_curr_profile().left_object_time3_sec;
		}
		void set_left_object_time3_sec(int left_object_time3_sec_)              //
		{
			get_curr_profile().left_object_time3_sec = left_object_time3_sec_;
		}

		bool get_show_left_objects() const                                      //
		{
			return get_curr_profile().show_left_objects;
		}
		void set_show_left_objects(bool show_left_objects_)                     //
		{
			get_curr_profile().show_left_objects = show_left_objects_;
		}

		bool get_show_trajectory() const                                        //
		{
			return get_curr_profile().show_trajectory;
		}
		void set_show_trajectory(bool show_trajectory_)                         //
		{
			get_curr_profile().show_trajectory = show_trajectory_;
		}

		void get_zones(zones_cont& zones_) const                                //
		{
			zones_ = get_curr_profile().zones;
		}
		void set_zones(const zones_cont& zones_)                                //
		{
			get_curr_profile().zones = zones_;
		}

		void get_lines(lines_cont& lines_) const                                //
		{
			lines_ = get_curr_profile().lines;
		}
		void set_lines(const lines_cont& lines_)                                //
		{
			get_curr_profile().lines = lines_;
		}

		void get_channel_name(cstring &channel_name_) const                     //
		{
			channel_name_ = channel_name;
		}
		void set_channel_name(const cstring &channel_name_)                     //
		{
			channel_name = channel_name_;
		}

		bool get_use_square_segmentation() const                                //
		{
			return get_curr_profile().use_square_segmentation;
		}
		void set_use_square_segmentation(bool use_square_segmentation_)         //
		{
			get_curr_profile().use_square_segmentation = use_square_segmentation_;
		}

		bool get_detect_patches_of_sunlight() const                             //
		{
			return get_curr_profile().detect_patches_of_sunlight;
		}
		void set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_)   //
		{
			get_curr_profile().detect_patches_of_sunlight = detect_patches_of_sunlight_;
		}

		bool get_cut_shadows() const                                            //
		{
			return get_curr_profile().cut_shadows;
		}
		void set_cut_shadows(bool cut_shadows_)                                 //
		{
			get_curr_profile().cut_shadows = cut_shadows_;
		}

		size_t get_curr_profile_ind() const                                     //
		{
			return curr_profile_ind;
		}
		void set_curr_profile_ind(size_t curr_profile_ind_)                     //
		{
			curr_profile_ind = (curr_profile_ind_ < profiles.size()) ? curr_profile_ind_ : 0;
		}

		void get_profiles(profiles_cont& profiles_) const                       //
		{
			profiles_ = profiles;
		}
		void set_profiles(const profiles_cont& profiles_)                       //
		{
			profiles = profiles_;
		}

		void set_curr_profile(const char* profile_name)                         // Задание текущего профиля по имени
		{
            std::string prname(profile_name);
			for (size_t i = 0; i < profiles.size(); ++i)
			{
                if (profiles[i].profile_name == prname)
				{
					curr_profile_ind = i;
					break;
				}
			}
		}
	};
	////////////////////////////////////////////////////////////////////////////
} // end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
