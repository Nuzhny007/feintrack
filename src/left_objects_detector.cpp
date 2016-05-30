#include "left_objects_detector.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::SimpleLeftObjectsDetector
///
SimpleLeftObjectsDetector::SimpleLeftObjectsDetector()
    :
      left_objects_count(0),
      left_object_time0(0),
      left_object_time1(0),
      left_object_time2(0),
      left_object_time3(0)
{
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::~SimpleLeftObjectsDetector
///
SimpleLeftObjectsDetector::~SimpleLeftObjectsDetector()
{

}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::SetLeftObjectTime1
/// \param left_object_time1_
///
void SimpleLeftObjectsDetector::SetLeftObjectTime0(int left_object_time0_)
{
    left_object_time0 = left_object_time0_;
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::SetLeftObjectTime1
/// \param left_object_time1_
///
void SimpleLeftObjectsDetector::SetLeftObjectTime1(int left_object_time1_)
{
    left_object_time1 = left_object_time1_;
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::SetLeftObjectTime2
/// \param left_object_time2_
///
void SimpleLeftObjectsDetector::SetLeftObjectTime2(int left_object_time2_)
{
    left_object_time2 = left_object_time2_;
}

////////////////////////////////////////////////////////////////////////////
void SimpleLeftObjectsDetector::SetLeftObjectTime3(int left_object_time3_)
{
    left_object_time3 = left_object_time3_;
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::Reset
/// \param hard
///
void SimpleLeftObjectsDetector::Reset()
{
    left_objects_count = 0;
    shady_left_objects.clear();
    lefted_objects.clear();
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::CheckLeftObject
/// \param obj
///
bool SimpleLeftObjectsDetector::CheckLeftObject(
        CTrackingObject& obj
        ) const
{
    return obj.get_left_frames() >= left_object_time1;
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::CheckShadyLeftObject
/// \param obj
///
void SimpleLeftObjectsDetector::CheckShadyLeftObject(
        CTrackingObject& obj
        )
{
    if (obj.get_left_frames() == left_object_time0)
    {
        CShadyLeftObj new_obj(obj.uid, obj.get_left_frames(), obj.get_left(), obj.get_right(), obj.get_top(), obj.get_bottom());
        // Если существует объект с центром в некоторой окрестности, то его удаляем а время складываем
        for (auto iter = shady_left_objects.begin(); iter != shady_left_objects.end();)
        {
            if ((abs(iter->rect.center_x() - obj.get_last_center_x()) < 2 * obj.get_left_epsilon()) &&
                    (abs(iter->rect.center_y() - obj.get_last_center_y()) < 2 * obj.get_left_epsilon()))
            {
                new_obj.life_time += iter->life_time;
                iter = shady_left_objects.erase(iter);
            }
            else
            {
                ++iter;
            }
        }
        shady_left_objects.push_back(new_obj);
        obj.set_left_frames(new_obj.life_time);
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::DelFromShadyLeftObjects
/// \param obj_uid
///
void SimpleLeftObjectsDetector::DelFromShadyLeftObjects(
        unsigned int obj_uid
        )
{
    for (auto iter = shady_left_objects.begin(); iter != shady_left_objects.end(); ++iter)
    {
        if (iter->obj_uid == obj_uid)
        {
            iter = shady_left_objects.erase(iter);
            return;
        }
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::DelUidFromShadyLeftObjects
/// \param obj_uid
///
void SimpleLeftObjectsDetector::DelUidFromShadyLeftObjects(
        unsigned int obj_uid
        )
{
    for (auto& obj : shady_left_objects)
    {
        if (obj.obj_uid == obj_uid)
        {
            obj.obj_uid = 0;
            return;
        }
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::IncTimeShadyLeftObjects
/// \param obj_uid
///
void SimpleLeftObjectsDetector::IncTimeShadyLeftObjects(
        unsigned int obj_uid
        )
{
    for (auto& obj : shady_left_objects)
    {
        if (obj.obj_uid == obj_uid)
        {
            obj.life_time++;
            return;
        }
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::AnalyzeShadyLeftObjects
///
void SimpleLeftObjectsDetector::AnalyzeShadyLeftObjects()
{
    for (auto iter = shady_left_objects.begin(); iter != shady_left_objects.end();)
    {
        if (!iter->obj_uid)
        {
            iter->not_detect_time++;
            if (iter->not_detect_time > left_object_time1)
                iter = shady_left_objects.erase(iter);
            else
                ++iter;
        }
        else
        {
            ++iter;
        }
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::NewLeftObject
/// \param obj_uid
/// \param region
///
void SimpleLeftObjectsDetector::NewLeftObject(
        unsigned int obj_uid,
        const CObjectRegion& region
        )
{
    lefted_objects.push_back(CLeftObjView(left_object_time3 - left_object_time1,
                                          region.get_left(), region.get_right(), region.get_top(), region.get_bottom(),
                                          obj_uid));
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::add_left_object_to_out_rects
/// \param left_obj
/// \param type
/// \param videoHeader
///
void SimpleLeftObjectsDetector::add_left_object_to_out_rects(
        const CLeftObjView &left_obj,
        CLeftObjRect::types type,
        const VideoHeader& videoHeader
        )
{
    // Здесь сделана попытка избежать лишних выделений памяти: массив left_obj_rects никогда не уменьшается
    // Он может только увеличивать свой размер, если на последующих кадрах выделенных объектов больше, чем на предыдущих
    if (++left_objects_count > left_obj_rects.size())
    {
        left_obj_rects.push_back(CLeftObjRect(left_obj.rect, type, left_obj.obj_uid));
        if (videoHeader.left_padding)
        {
            left_obj_rects[left_objects_count - 1].left += videoHeader.left_padding;
            left_obj_rects[left_objects_count - 1].right += videoHeader.left_padding;
        }
        if (videoHeader.top_padding)
        {
            left_obj_rects[left_objects_count - 1].top += videoHeader.top_padding;
            left_obj_rects[left_objects_count - 1].bottom += videoHeader.top_padding;
        }
    }
    else
    {
        left_obj_rects[left_objects_count - 1].left = left_obj.rect.left + videoHeader.left_padding;
        left_obj_rects[left_objects_count - 1].right = left_obj.rect.right + videoHeader.left_padding;
        left_obj_rects[left_objects_count - 1].top = left_obj.rect.top + videoHeader.top_padding;
        left_obj_rects[left_objects_count - 1].bottom = left_obj.rect.bottom + videoHeader.top_padding;
        left_obj_rects[left_objects_count - 1].type = type;
        left_obj_rects[left_objects_count - 1].obj_uid = left_obj.obj_uid;
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::AnalyzeLeftedObjects
/// \param videoHeader
///
void SimpleLeftObjectsDetector::AnalyzeLeftedObjects(
        const VideoHeader& videoHeader
        )
{
    left_objects_count = 0;

    // Если объекты пересекаются, то более старый удаляем
    lefted_objects.sort(CLeftObjView::bigger);
    for (std::list<CLeftObjView>::iterator iter1 = lefted_objects.begin(); iter1 != lefted_objects.end(); ++iter1)
    {
        std::list<CLeftObjView>::iterator iter2 = iter1;
        ++iter2;
        for (; iter2 != lefted_objects.end();)
        {
            if (segments_superposition(iter1->rect.left, iter1->rect.right, iter2->rect.left, iter2->rect.right) &&
                    segments_superposition(iter1->rect.top, iter1->rect.bottom, iter2->rect.top, iter2->rect.bottom))
            {
                iter2 = lefted_objects.erase(iter2);
            }
            else
            {
                ++iter2;
            }
        }
    }

    // Заполняем массив для обводки
    for (std::list<CLeftObjView>::iterator iter = lefted_objects.begin(); iter != lefted_objects.end();)
    {
        // Удаляем оставленные предметы, жизнь которых больше определённого времени (left_object_life_time)
        if (iter->life_time < 1)
        {
            iter = lefted_objects.erase(iter);
        }
        else
        {
            // Уменьшаем время жизни объекта
            iter->life_time--;

            // Добавляем в массив оставленных объектов
            add_left_object_to_out_rects(*iter, ((left_object_time3 - iter->life_time < left_object_time2)? CLeftObjRect::first: CLeftObjRect::second), videoHeader);
            ++iter;
        }
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief SimpleLeftObjectsDetector::GetLeftObjects
/// \param rect_arr
/// \param rect_count
///
void SimpleLeftObjectsDetector::GetLeftObjects(
        const CLeftObjRect* &rect_arr,
        size_t& rect_count
        ) const
{
    rect_count = left_objects_count;
    if (rect_count)
    {
        rect_arr = &left_obj_rects[0];
    }
}

////////////////////////////////////////////////////////////////////////////

} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
