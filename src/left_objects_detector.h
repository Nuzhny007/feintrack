#pragma once
#include "feintrack_objects.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
////////////////////////////////////////////////////////////////////////////
/// \brief The LeftObjectsDetector class
///
class LeftObjectsDetector
{
public:
    LeftObjectsDetector()
    {
    }

    virtual ~LeftObjectsDetector()
    {
    }

    virtual void SetLeftObjectTime0(int left_object_time0_) = 0;
    virtual void SetLeftObjectTime1(int left_object_time1_) = 0;
    virtual void SetLeftObjectTime2(int left_object_time2_) = 0;
    virtual void SetLeftObjectTime3(int left_object_time3_) = 0;

    virtual void Reset() = 0;

    virtual void AnalyzeLeftedObjects(const VideoHeader& videoHeader) = 0;
    virtual void AnalyzeShadyLeftObjects() = 0;

    virtual bool CheckLeftObject(CTrackingObject& obj) const = 0;
    virtual void CheckShadyLeftObject(CTrackingObject& obj) = 0;

    virtual void DelFromShadyLeftObjects(unsigned int obj_uid) = 0;
    virtual void DelUidFromShadyLeftObjects(unsigned int obj_uid) = 0;
    virtual void IncTimeShadyLeftObjects(unsigned int obj_uid) = 0;

    virtual void NewLeftObject(unsigned int obj_uid, const CObjectRegion& region) = 0;

    virtual void GetLeftObjects(const CLeftObjRect* &rect_arr, size_t& rect_count) const = 0;
};

////////////////////////////////////////////////////////////////////////////
/// \brief The SimpleLeftObjectsDetector class
///
class SimpleLeftObjectsDetector : public LeftObjectsDetector
{
public:
    SimpleLeftObjectsDetector();
    ~SimpleLeftObjectsDetector();

    void SetLeftObjectTime0(int left_object_time0_);
    void SetLeftObjectTime1(int left_object_time1_);
    void SetLeftObjectTime2(int left_object_time2_);
    void SetLeftObjectTime3(int left_object_time3_);

    void Reset();

    void AnalyzeLeftedObjects(const VideoHeader& videoHeader); // Анализ оставленных предметов
    void AnalyzeShadyLeftObjects();                            // Удаление объектов, не обнаруживающихся в течение некоторого времени

    bool CheckLeftObject(CTrackingObject& obj) const;          // Проверка на то, является ли объект оставленным
    void CheckShadyLeftObject(CTrackingObject& obj);           // Добавление объекта к списку объектов, возможно являющихся оставленными предметами

    void DelFromShadyLeftObjects(unsigned int obj_uid);        // Удаление объекта из списка объектов, возможно являющихся оставленными предметами
    void DelUidFromShadyLeftObjects(unsigned int obj_uid);     // Удаление идентификатора объекта из списка объектов, возможно являющихся оставленными предметами
    void IncTimeShadyLeftObjects(unsigned int obj_uid);        // Увеличение времени жизни объекта

    void NewLeftObject(unsigned int obj_uid, const CObjectRegion& region); // Добавление нового объекта в список оставленых предметов

    void GetLeftObjects(const CLeftObjRect* &rect_arr, size_t& rect_count) const;

private:
    std::vector<CLeftObjRect> left_obj_rects;             // Массив координат оставленных объектов. Заполняется для рисования прямоугольников. Валиден до прихода следующего кадра
    size_t left_objects_count;                            // Количество оставленных объектов

    std::list<CLeftObjView> lefted_objects;               // Оставленные предметы

    int left_object_time0;                                // Время, после которого объект заносится в список объектов, возможно являющихся оставленными
    int left_object_time1;                                // Время, после которого объект считается похожим на оставленный
    int left_object_time2;                                // Время, после которого объект считается оставленным
    int left_object_time3;                                // Время, после которого оставленный предмет удаляется

    std::list<CShadyLeftObj> shady_left_objects;          // Объекты, возможно являющиеся оставленными предметами

    void add_left_object_to_out_rects(const CLeftObjView &left_obj, CLeftObjRect::types type, const VideoHeader& videoHeader); // Добавление оставленного предмета на вывод
};
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
