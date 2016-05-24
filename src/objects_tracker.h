#pragma once
#include "feintrack_objects.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
////////////////////////////////////////////////////////////////////////////
typedef std::basic_string<char> mstring;

////////////////////////////////////////////////////////////////////////////
/// \brief The ObjectsTracker class
///
class ObjectsTracker
{
public:
    ObjectsTracker()
    {

    }

    virtual ~ObjectsTracker()
    {

    }

    virtual void SetFps(int new_fps, int left_object_time1_sec, int left_object_time2_sec, int left_object_time3_sec) = 0;

    virtual void SetLeftObjectTime1(int left_object_time1_) = 0;
    virtual void SetLeftObjectTime2(int left_object_time2_) = 0;
    virtual void SetLeftObjectTime3(int left_object_time3_) = 0;

    virtual void Reset(bool hard) = 0;

    virtual void Track(regions_container& regions, regions_container& update_regions, const VideoHeader& videoHeader, int selection_time, bool show_trajectory, const zones_cont& zones, const lines_cont& lines) = 0;

    virtual void GetObjects(CObjRect* &rect_arr, size_t& rect_count) = 0;

    virtual void SetOneObject(unsigned int uid, int left, int right, int top, int bottom) = 0;
    virtual void GetDelObjects(unsigned int* &uids_arr, size_t& uids_count) = 0;

    virtual bool GetObject(size_t obj_ind, CObjRect& objRect) = 0;
    virtual void GetLeftObjects(CLeftObjRect* &rect_arr, size_t& rect_count) = 0;
};

////////////////////////////////////////////////////////////////////////////
/// \brief The NativeTracker class
///
class NativeTracker : public ObjectsTracker
{
public:
    NativeTracker();
    ~NativeTracker();

    void SetFps(int new_fps, int left_object_time1_sec, int left_object_time2_sec, int left_object_time3_sec);

    void SetLeftObjectTime1(int left_object_time1_);
    void SetLeftObjectTime2(int left_object_time2_);
    void SetLeftObjectTime3(int left_object_time3_);

    void Reset(bool hard);

    void Track(regions_container& regions, regions_container& update_regions, const VideoHeader& videoHeader, int selection_time, bool show_trajectory, const zones_cont& zones, const lines_cont& lines);

    void GetObjects(CObjRect* &rect_arr, size_t& rect_count);

    void SetOneObject(unsigned int uid, int left, int right, int top, int bottom);
    void GetDelObjects(unsigned int* &uids_arr, size_t& uids_count);

    bool GetObject(size_t obj_ind, CObjRect& objRect);
    void GetLeftObjects(CLeftObjRect* &rect_arr, size_t& rect_count);

private:
    objects_container objects_history;                    // Список объектов, найденных на предыдущих кадрах

    std::vector<CObjRect> obj_rects;                      // Массив координат объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
    size_t objects_count;                                 // Количество найденных объектов на последнем кадре

    std::vector<CLeftObjRect> left_obj_rects;             // Массив координат оставленных объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
    size_t left_objects_count;                            // Количество оставленных объектов

    std::list<CLeftObjView> lefted_objects;               // Оставленные предметы
    void analyze_lefted_objects(const VideoHeader& videoHeader); // Анализ оставленных предметов

    int left_object_time0;                                // Время, после которого объект заносится в список объектов, возможно являющихся оставленными
    int left_object_time1;                                // Время, после которого объект считается похожим на оставленный
    int left_object_time2;                                // Время, после которого объект считается оставленным
    int left_object_time3;                                // Время, после которого оставленный предмет удаляется

    void add_uid_to_del_objects(unsigned int uid);        // Добавление иденфификатора с списку удалёных объектов
    void del_object(std::unique_ptr<CTrackingObject>& object, bool del_adv_data); // Удаление объекта и связанных с ним данных
    std::vector<unsigned int> del_objects;                // Массив координат объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
    size_t del_objects_count;                             // Количество найденных объектов на последнем кадре

    unsigned int get_free_uid();                          // Получение свободного иденификатора объекта
    unsigned int lastUid;                                 // Наибольший идентификатор объекта

    regions_container::iterator find_region_by_center(regions_container& regions, int c_x, int c_y, int width, int height); // Поиск подходящего региона по координатам центра объекта

    float_t weight_threshold;                             // Порог. Объекты с весом ниже него признаются несущественными или устаревшими и удаляются
    float_t weight_alpha;                                 // Коэффициент, отвечающий за величину обучения веса региона на каждом шаге

    std::list<CShadyLeftObj> shady_left_objects;          // Объекты, возможно являющиеся оставленными предметами
    void add_to_shady_left_objects(CTrackingObject &obj); // Добавление объекта к списку объектов, возможно являющихся оставленными предметами
    void del_from_shady_left_objects(unsigned int obj_uid);      // Удаление объекта из списка объектов, возможно являющихся оставленными предметами
    void del_uid_from_shady_left_objects(unsigned int obj_uid);  // Удаление идентификатора объекта из списка объектов, возможно являющихся оставленными предметами
    void inc_time_shady_left_objects(unsigned int obj_uid);      // Увеличение времени жизни объекта
    void analyze_shady_left_objects();                    // Удаление объектов, не обнаруживающихся в течение некоторого времени

    objects_container::iterator get_object_by_region(const CObjectRegion& region, objects_container::iterator from_obj); //Получение объекта, наиболее приближённого к данному региону

    template<class T>
    void add_object_to_out_rects(const T &rect, const CTrackingObject &object, object_types obj_type, const mstring &zone_name,
                                 const VideoHeader& videoHeader,
                                 bool show_trajectory
                                 ); // Добавление объекта на вывод

    void add_left_object_to_out_rects(const CLeftObjView &left_obj, CLeftObjRect::types type, const VideoHeader& videoHeader); // Добавление оставленного предмета на вывод

    template<typename ZONES_T, typename T>
    bool is_in_zone(const ZONES_T& zones, const T& rect, mstring* zone_name) const; // Попадает ли прямоугольник в список зон детекции

    bool with_line_intersect(const lines_cont& lines, const CTrackingObject& obj, int new_center_x, int new_center_y, const VideoHeader& videoHeader); // Проверка пересечения объектом линии
};
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
