#pragma once
#include <deque>
#include <memory>

#include "feintrack_dll.h"
#include "utils.h"
////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	//Использовать линейное предсказание положения объекта (Иначе используется также линейный алгоритм но с предварительной обработкой траектории алгоритмом Дугласа-Пекера)
#define LIN_MNK 0
	////////////////////////////////////////////////////////////////////////////
	
	// Хранит информацию о регионе, ограничивающем объект
	class CObjectRegion
	{
	private:
		int left;             //Левая,
		int right;            //правая,
		int top;              //верхняя
		int bottom;           //и нижняя координаты прямоугольника, ограничивающего регион

		size_t obj_pxl_count; //Количество пикселей объекта в регионе

		static const int region_h_granz = 3; //Толщина границы региона слева и справа
		static const int region_v_granz = 4; //Толщина границы региона сверху и снизу

	public:
		CObjectRegion();
		CObjectRegion(int left_, int right_, int top_, int bottom_, size_t obj_pxl_count_ = 1);
		CObjectRegion(const CObjectRegion& region);
		~CObjectRegion();

		int get_left() const;               //Функции, возвращающие границы региона
		int get_right() const;              //
		int get_top() const;                //
		int get_bottom() const;             //

		void set_left(int left_);           //Функции, задающие границы региона
		void set_right(int right_);         //
		void set_top(int top_);             //
		void set_bottom(int bottom_);       //

		int width() const;                  // Возвращает ширину
		int height() const;                 // и высоту прямоугольника, ограничивающего регион

		//Проверка: лежит ли точка с указанными координатами внутри региона, на или рядом с его границами
		bool in_region(int x, int y) const;

		//Если рядом лежащая точка с указанными координатами не принадлежит региону, то он увеличивает свои размеры
		void add_point(int x, int y);

		void regions_merging(const CObjectRegion& region); //Объединение 2-х регионов

		//Проверка: лежит ли прямоугольник с указанными координатами рядом с регионом
		bool near_region(int left_, int top_) const;

		//Добавление прямоугольника к региону (добавляемый прямоугольник находится либо справа, либо снизу)
		void add_rect(int right_, int bottom_, size_t obj_pxl_count_);

		int get_center_x() const;           //Возвращают координаты центра региона
		int get_center_y() const;           //

		float_t density() const;            //Возвращает отношение точек объекта в регионе к общему количеству точек

		void resize_to_max_granz(int max_width, int max_height); //Расширяет свои границы с учётом значений region_h_granz и region_v_granz

		static bool size_bigger(const CObjectRegion& reg1, const CObjectRegion& reg2); //Функция для сортировки регионов по размеру
	};
	////////////////////////////////////////////////////////////////////////////
	
	// Хранит информацию об объектах, оставшихся от предыдущих кадров
	class CTrackingObject
	{
	private:
		typedef std::deque<POINT_<int> > traectory_cont;

		int center_x;                                     //Координаты последнего центра объекта
		int center_y;                                     //

		RECT_ rect;                                       //Прямоугольник с последними размерами объекта

		int dx;                                           //Величина последнего смещеня объекта по горизонтали
		int dy;                                           //и по вертикали

		int left_center_x;                                //Последние координаты центра оставленного предмета
		int left_center_y;                                //
		int frames_in_eps;                                //Количество кадров, на которых центр объекта не вышел за границы left_eps
		int left_epsilon;                                 //Максимальная величина смещения цетра объекта, при превышении которой объект перестаёт быть оставленным

		int obj_recogn_count;                             //Количество положительных распознаваний объекта как человека
		object_types type;                                //Текущий тип объекта
		static const int min_recogn_count = 5;            //Минимальное количество положительных распознаваний, при которых объект относится к данному типу

		CTrackingObject *merge_object1;                   //Объекты,
		CTrackingObject *merge_object2;                   //с которыми было произведено слияние при пересечении траекторий

#if LIN_MNK
		static const size_t STAT_FRAME_COUNT = 24;        //Количество кадров, которые используются для предсказания будущего положения объекта
		cyclic_array<POINT_<int>, STAT_FRAME_COUNT> stat; //Координаты объекта на предыдущих кадрах
		size_t coords_collected;                          //Количество уже собранных координат
		float_t kx, bx, ky, by;                           //Коэффициенты уравнений траектории движения объекта
#else
		static const int dp_epsilon = 5;                  // Точность работы алгоритма Дугласа-Пекера (в пикселях)
		traectory_cont dp_traectory_x;                    // Траектория движения объекта по горизонтали и
		traectory_cont dp_traectory_y;                    // по вертикали после обработки её алгоритмом Дугласа-Пекера

		int predict(const traectory_cont& traectory, const traectory_cont& orig_traectory, int delta_time) const; // Предсказание положения объекта на указанный промежуток времени в кадрах
#endif

		traectory_cont traectory_x;                       //Полная траектория движения объекта по горизонтали и
		traectory_cont traectory_y;                       //по вертикали

	public:
        CTrackingObject(int center_x_, int center_y_, unsigned int uid_);
		CTrackingObject(const CTrackingObject &obj);
		~CTrackingObject();

		object_types get_type() const;                    //Получение типа объекта
		void set_new_type(object_types new_type);         //Задание распознанного на очередном кадре типа объекта

		bool have_merge_object() const;                          //Есть ли объект, с которым произвелось слияние
		void add_merge_obj(const CTrackingObject &merge_object); //Слияние с объектом
		CTrackingObject *get_merge_object(size_t ind);           //Получение объекта, с которым было произведено слияние
		void inc_merge_frames();                                 //Увеличение времени слияния на 1
		void set_merge_objects_to_null();                        //Обнулить указатели на объекты
        bool has_merge_object(unsigned int object_uid) const;           //Имеется ли объект с искомым uid

		static const float_t default_weight;              //Первоначальное значение веса объекта

		int get_x_future_val(int future_time) const;      //Получение оценки координаты x нового центра объекта через future_time кадров
		int get_y_future_val(int future_time) const;      //Получение оценки координаты y нового центра объекта через future_time кадров

		int get_new_center_x() const;                     //Получение приблизительной оценки координаты x нового центра объекта на основании имеющихся данных
		int get_new_center_y() const;                     //Получение приблизительной оценки координаты y нового центра объекта на основании имеющихся данных

		int get_last_center_x() const;                    //Получение последней координаты x центра объекта
		int get_last_center_y() const;                    //Получение последней координаты y центра объекта

		int get_left_frames() const;                      //Получение и
		void set_left_frames(int frames_in_eps_);         //задание количества кадров, на которых величина смещения объекта меньше заданной
		int get_left_epsilon() const;                     //Возвращает максимально возможную величину отклонения центра объекта при определении оставленных предметов

		void set_last_center(int new_center_x, int new_center_y); //Задание координат последнего местонахождения центра объекта
		void set_rect(int left, int right, int top, int bottom);  //Задание новых координат объекта

		//Получение последних детектируемых координат объекта
		const RECT_ &get_rect() const;
		int get_left() const;
		int get_right() const;
		int get_top() const;
		int get_bottom() const;

		int width() const;  //Получение ширины и
		int height() const; //высоты объекта

		void recalc_center(); //Пересчитывает координаты центра объекта при отсутствии информации о его местоположении на последнем кадре

		void get_traectory(CObjRect &obj_rect, uint frame_width, uint frame_height, int left_padding, int top_padding) const; //Получение траектории движения объекта

        unsigned int uid;            //Идентификатор (номер) объекта

		float_t weight;       //Вес - величина, лежащая на промежутке (0; 1) и определяющая актуальность данного объекта
		int life_time;        //Время жизни объекта в кадрах
		int frames_left;      //Количество кадров, на которых объект не был найден

		//Функции для сортировки объектов по весу, времени жизни и комплексного (вес + временя жизни) показателя
		static bool weight_bigger(const CTrackingObject &obj1, const CTrackingObject &obj2);
        static bool life_bigger(const std::unique_ptr<CTrackingObject>& obj1, const std::unique_ptr<CTrackingObject>& obj2);
		static bool weight_life_bigger(const CTrackingObject &obj1, const CTrackingObject &obj2);
	};
	////////////////////////////////////////////////////////////////////////////
	
	// Хранит информацию об оставленном предмете
	struct CLeftObjView
	{
		CLeftObjView()
			: life_time(0)
		{}
		CLeftObjView(int life_time_, int left, int right, int top, int bottom)
			: life_time(life_time_)
		{
			rect.left = left;
			rect.right = right;
			rect.top = top;
			rect.bottom = bottom;
		}
		int life_time; //Время жизни предмета
		RECT_ rect;    //Прямоугольник предмета

		//Сравнение по времени жизни
		static bool bigger(const CLeftObjView &obj1, const CLeftObjView &obj2)
		{
			return obj1.life_time > obj2.life_time;
		}
	};
	////////////////////////////////////////////////////////////////////////////
	
	// Объект, возможно являющийся оставленным
	struct CShadyLeftObj: public CLeftObjView
	{
		CShadyLeftObj()
			: real_obj_uid(0), not_detect_time(0)
		{
		}
        CShadyLeftObj(unsigned int real_obj_uid_, int life_time_, int left, int right, int top, int bottom)
			: CLeftObjView(life_time_, left, right, top, bottom), real_obj_uid(real_obj_uid_), not_detect_time(0)
		{
		}
        unsigned int real_obj_uid;  //Идентификатор объекта, который возможно является оставленным
		int not_detect_time; //Количество кадров, на которых объект не был найден
	};
	////////////////////////////////////////////////////////////////////////////

	typedef std::list<CObjectRegion> regions_container;
    typedef std::list<std::unique_ptr<CTrackingObject> > objects_container;
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
