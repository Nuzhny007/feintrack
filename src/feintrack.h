#pragma once
#include <map>
#include <list>
#include <limits>

#include "feintrack_params.h"
#include "feintrack_objects.h"
#include "bgrnd_substr.h"
#include "segmentation.h"
#include "recognition.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
	////////////////////////////////////////////////////////////////////////////

	typedef std::basic_string<char> mstring;
	////////////////////////////////////////////////////////////////////////////

	// Выделение и сопровождение объектов
	class CFeinTrack
	{
	private:
		static const float_t density_threshold;  // Порог плотности региона. Регионы с уровнем плотности ниже этого порога сразу удаляются

		bool use_square_segmentation;            // Использовать сегментацию объектов как прямоугольных регионов. Иначе используется 8-ми связность

		int left_object_time0;                   // Время, после которого объект заносится в список объектов, возможно являющихся оставленными
		int left_object_time1;                   // Время, после которого объект считается похожим на оставленный
		int left_object_time2;                   // Время, после которого объект считается оставленным
		int left_object_time3;                   // Время, после которого оставленный предмет удаляется

		int left_object_time1_sec;               // Время в секундах, после которого объект считается похожим на оставленный
		int left_object_time2_sec;               // Время в секундах, после которого объект считается оставленным
		int left_object_time3_sec;               // Время в секундах, после которого оставленный предмет удаляется

        uint32_t frame_width;                    // Ширина
        uint32_t frame_height;                   // и высота одного кадра в пикселях
		color_type curr_color_type;              // Текущий тип цветового пространства анализируемого кадра

		int fps;                                 // Количество кадров в секунду, поступающих на фильтр. Обновление статистики производится раз в секунду. Время будет измеряться по кадрам.
		int selection_time;                      // Временной порог селекции объектов
		int curr_frame;                          // Номер текущего кадра

		bool cut_shadows;                        // Убирать тени, основываясь на анализе формы объекта

		bool use_cuda;                           // Использовать CUDA для вычитания фона и морфологии
		int cuda_device_ind;                     // Индекс используемого устройства, поддерживающего CUDA

        int pixel_size;                          // Размер одного пикселя в байтах

		bool use_morphology;                     // Использовать операцию математической морфологии "открытие" для результатов вычитания фона

		int min_region_width;                    // Минимальная ширина и
		int min_region_height;                   // высота региона

		int left_padding;                        // Отступ от левого и
		int top_padding;                         // верхнего угла кадра при наличии ограничения
		RECT_ analyze_area;                      // Часть кадра, которая будет анализироваться

		bool need_background_update;             // Обновлять ли модель заднего плана

		regions_container regions;                                // Список регионов на последнем кадре
        void regions_preprocessing(const uchar* buf, uint32_t pitch); // Предварительный анализ и обработка регионов: отсечение теней, удаление маленьких регионов и т.д.
#if !ADV_OUT
        void tracking_objects(const uchar* buf, uint32_t pitch);      // Анализ регионов и добавление подходящих на вывод
#else
        void tracking_objects(const uchar* buf, uint32_t pitch, uchar* adv_buf_rgb24); // Анализ регионов
#endif

		objects_container objects_history;                    // Список объектов, найденных на предыдущих кадрах

        void add_uid_to_del_objects(unsigned int uid);               // Добавление иденфификатора с списку удалёных объектов
        void del_object(std::unique_ptr<CTrackingObject>& object, bool del_adv_data); // Удаление объекта и связанных с ним данных
        std::vector<unsigned int> del_objects;                       // Массив координат объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
		size_t del_objects_count;                             // Количество найденных объектов на последнем кадре

        unsigned int get_free_uid() const;                           // Получение свободного иденификатора объекта

		regions_container::iterator find_region_by_center(int c_x, int c_y, int width, int height); // Поиск подходящего региона по координатам центра объекта
        regions_container::iterator find_region_by_hist(const uchar* buf, int pitch, const std::unique_ptr<CTrackingObject>& obj);  // Поиск региона по гистограмме

		float_t weight_threshold;                             // Порог. Объекты с весом ниже него признаются несущественными или устаревшими и удаляются
		float_t weight_alpha;                                 // Коэффициент, отвечающий за величину обучения веса региона на каждом шаге

		std::vector<CObjRect> obj_rects;                      // Массив координат объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
		size_t objects_count;                                 // Количество найденных объектов на последнем кадре

		std::list<CShadyLeftObj> shady_left_objects;          // Объекты, возможно являющиеся оставленными предметами
		void add_to_shady_left_objects(CTrackingObject &obj); // Добавление объекта к списку объектов, возможно являющихся оставленными предметами
        void del_from_shady_left_objects(unsigned int obj_uid);      // Удаление объекта из списка объектов, возможно являющихся оставленными предметами
        void del_uid_from_shady_left_objects(unsigned int obj_uid);  // Удаление идентификатора объекта из списка объектов, возможно являющихся оставленными предметами
        void inc_time_shady_left_objects(unsigned int obj_uid);      // Увеличение времени жизни объекта
		void analyze_shady_left_objects();                    // Удаление объектов, не обнаруживающихся в течение некоторого времени

		std::vector<CLeftObjRect> left_obj_rects;             // Массив координат оставленных объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
		size_t left_objects_count;                            // Количество оставленных объектов
		bool show_left_objects;                               // Подавать на отображение оставленные предметы

		bool show_objects;                                    // Отображать объекты
		bool show_trajectory;                                 // Отображать траектории объектов

		std::list<CLeftObjView> lefted_objects;               // Оставленные предметы
		void analyze_lefted_objects();                        // Анализ оставленных предметов

		lines_cont lines;                                     // Список пользовательских линий с реальными координатами на кадре
		lines_cont correct_lines;                             // Список пользовательских линий с координатами относительно области анализа
		void recalc_correct_lines();                          // Пересчёт координат линий на кадре

        bool with_line_intersect(const CTrackingObject &obj, int new_center_x, int new_center_y); // Проверка пересечения объектом линии

		zones_cont zones;                                     // Список зон детекции с реальными координатами на кадре
		zones_cont correct_zones;                             // Список зон детекции с координатами относительно области анализа
		void recalc_correct_zones();                          // Пересчёт координат зон на кадре
		template<class T> bool is_in_zone(const T &rect, mstring *zone_name) const; // Попадает ли прямоугольник в список зон детекции
		template<class T> const CZone* get_zone(const T &rect) const;               // Получение зоны, в которую попадает прямоугольник

		template<class T> void add_object_to_out_rects(const T &rect, const CTrackingObject &object, object_types obj_type, const mstring &zone_name); // Добавление объекта на вывод
		void add_left_object_to_out_rects(const CLeftObjView &left_obj, CLeftObjRect::types type); // Добавление оставленного предмета на вывод

        bool use_recognition;                                // Использовать распознавание объектов

        bool cut_shadow(CObjectRegion& region);              // Отсечение тени

		objects_container::iterator get_object_by_region(const CObjectRegion& region, objects_container::iterator from_obj); //Получение объекта, наиболее приближённого к данному региону

        bgrnd_substr_types bgrnd_type;                       // Тип алгоритма вычитания фона

        std::unique_ptr<CBackSubstraction> back_substractor; // Вычитание фона
        CSegmentation segmentator;                           // Сегментация объектов переднего плана в регионы
        CRecognition recognizer;                             // Распознавание объектов

	public:
		CFeinTrack();
		~CFeinTrack();

#if !ADV_OUT
        int new_frame(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, color_type buf_type); // Анализ очередного кадра
#else
        int new_frame(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24); // Анализ очередного кадра
#endif

		void set_sensitivity(int sens_level);                           // Задаёт уровень чувствительности для вычитания фона (от 1 до 100)
		int get_sensitivity() const;                                    // Получение уровня чувствительности для вычитания фона (от 1 до 100)

		void get_objects(CObjRect* &rect_arr, size_t& rect_count);      // Получение списка отображаемых объектов, обнаруженных на последнем кадре
        void set_one_object(unsigned int uid, int left, int right, int top, int bottom); // Задание координат единственного объекта, который будет отображаться на кадре
		bool get_object_points(size_t obj_ind, POINTF* points, size_t& max_points); // Получение набора точек, принадлежащих объекту
        void get_del_objects(unsigned int* &uids_arr, size_t& uids_count);     // Получение списка идентификаторов удалённых на последнем кадре объектов
		void get_left_objects(CLeftObjRect* &rect_arr, size_t& rect_count); // Получение списка оставленных объектов

		int get_fps() const;                                            // Получение и
		void set_fps(int new_fps);                                      // задание fps

		void set_show_objects(bool new_val);                            // Показывать/не показывать объекты
		bool get_show_objects() const;                                  // Получение значения

		void set_zones_list(const zones_cont& zones_);                  // Задание и
		void get_zones_list(zones_cont& zones_) const;                  // получение списка зон

		void set_show_left_objects(bool show_left_objects_);            // Задание и
		bool get_show_left_objects() const;                             // получение значения show_left_objects

		void set_show_trajectory(bool show_trajectory_);                // Задание и
		bool get_show_trajectory() const;                               // получение значения show_trajectory

		bool get_use_morphology() const;                                // Задание и
		void set_use_morphology(bool use_morphology_);                  // получение значения use_morphology

		void set_lines_list(const lines_cont& lines_);                  // Задание и
		void get_lines_list(lines_cont& lines_) const;                  // получение списка линий

		bool get_use_square_segmentation() const;                        // Получение и
		void set_use_square_segmentation(bool use_square_segmentation_); // задание режима сегментации

		bool get_use_recognition() const;                           // Получение и
		void set_use_recognition(bool new_val);                     // задание значения use_recognition

		int get_min_region_width() const;                           // Получение и
		void set_min_region_width(int min_region_width_);           // задание минимального размера региона
		int get_min_region_height() const;                          // Получение и
		void set_min_region_height(int min_region_height_);         // задание минимального размера региона

		int get_selection_time() const;                             // Получение и
		void set_selection_time(int selection_time_);               // задание времени фильтрации новых объектов

		RECT_ get_analyze_area() const;                             // Получение и
		void set_analyze_area(const RECT_ &analyze_area_);          // задание области анализа изображения

		int get_left_object_time1_sec() const;                      // Получение и
		void set_left_object_time1_sec(int left_object_time1_sec_); // задание времени детекции оставленных предметов
		int get_left_object_time2_sec() const;                      // Получение и
		void set_left_object_time2_sec(int left_object_time2_sec_); // задание времени детекции оставленных предметов
		int get_left_object_time3_sec() const;                      // Получение и
		void set_left_object_time3_sec(int left_object_time3_sec_); // задание времени детекции оставленных предметов

		void enable_back_update(bool enable_val);                   // Разрешение/запрещение обновления фона

		bool get_detect_patches_of_sunlight() const;                           // Используется ли детектор бликов
		void set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_); // Включение/выключение детектора бликов

		bool get_cut_shadows() const;                               // Используется ли отсечение теней
		void set_cut_shadows(bool cut_shadows_);                    // Включение/выключение отсечения теней

		void set_use_cuda(bool use_cuda_, int cuda_device_ind_);    // Задание и
		bool get_use_cuda() const;                                  // получение использования CUDA
		int get_cuda_device() const;                                // Получение индекса используемого устройства, поддерживающего CUDA

		void set_bgrnd_type(bgrnd_substr_types bgrnd_type_);        // Задание и
		bgrnd_substr_types get_bgrnd_type() const;                  // получение типа используемого алгоритма вычитания фона
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
