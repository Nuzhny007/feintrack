#pragma once
#include "feintrack_objects.h"
#include "left_objects_detector.h"

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

    virtual void GetObjects(const CObjRect* &rect_arr, size_t& rect_count) const = 0;

    virtual void SetOneObject(unsigned int uid, int left, int right, int top, int bottom) = 0;
    virtual void GetDelObjects(unsigned int* &uids_arr, size_t& uids_count) = 0;

    virtual bool GetObject(size_t obj_ind, CObjRect& objRect) const = 0;
    virtual void GetLeftObjects(const CLeftObjRect* &rect_arr, size_t& rect_count) const = 0;
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

    void GetObjects(const CObjRect* &rect_arr, size_t& rect_count) const;

    void SetOneObject(unsigned int uid, int left, int right, int top, int bottom);
    void GetDelObjects(unsigned int* &uids_arr, size_t& uids_count);

    bool GetObject(size_t obj_ind, CObjRect& objRect) const;
    void GetLeftObjects(const CLeftObjRect* &rect_arr, size_t& rect_count) const;

private:
    objects_container objects_history;                    // Список объектов, найденных на предыдущих кадрах

    std::vector<CObjRect> obj_rects;                      // Массив координат объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
    size_t objects_count;                                 // Количество найденных объектов на последнем кадре

    void add_uid_to_del_objects(unsigned int uid);        // Добавление иденфификатора с списку удалёных объектов
    void del_object(std::unique_ptr<CTrackingObject>& object, bool del_adv_data); // Удаление объекта и связанных с ним данных
    std::vector<unsigned int> del_objects;                // Массив координат объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
    size_t del_objects_count;                             // Количество найденных объектов на последнем кадре

    unsigned int get_free_uid();                          // Получение свободного иденификатора объекта
    unsigned int lastUid;                                 // Наибольший идентификатор объекта

    regions_container::iterator find_region_by_center(regions_container& regions, int c_x, int c_y, int width, int height); // Поиск подходящего региона по координатам центра объекта

    float_t weight_threshold;                             // Порог, объекты с весом ниже которого признаются несущественными или устаревшими и удаляются
    float_t weight_alpha;                                 // Коэффициент, отвечающий за величину обучения веса региона на каждом шаге

    SimpleLeftObjectsDetector left_detector;              // Детектор оставленных предметов

    objects_container::iterator get_object_by_region(const CObjectRegion& region, objects_container::iterator from_obj); //Получение объекта, наиболее приближённого к данному региону

    template<class T>
    void add_object_to_out_rects(const T &rect, const CTrackingObject &object, const mstring &zone_name,
                                 const VideoHeader& videoHeader,
                                 bool show_trajectory
                                 ); // Добавление объекта на вывод

    template<typename ZONES_T, typename T>
    bool is_in_zone(const ZONES_T& zones, const T& rect, mstring* zone_name) const; // Попадает ли прямоугольник в список зон детекции

    bool with_line_intersect(const lines_cont& lines, const CTrackingObject& obj, int new_center_x, int new_center_y, const VideoHeader& videoHeader); // Проверка пересечения объектом линии
};
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
