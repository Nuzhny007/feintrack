#include "objects_tracker.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::NativeTracker
///
NativeTracker::NativeTracker()
    :
      objects_count(0),
      del_objects_count(0),
      lastUid(1),
      weight_threshold(0.1f),
      weight_alpha(0.5f / 25)
{
    obj_rects.reserve(1500);
    del_objects.reserve(50);
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::~NativeTracker
///
NativeTracker::~NativeTracker()
{

}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::SetFps
/// \param new_fps
/// \param left_object_time1_sec
/// \param left_object_time2_sec
/// \param left_object_time3_sec
///
void NativeTracker::SetFps(
        int new_fps,
        int left_object_time1_sec,
        int left_object_time2_sec,
        int left_object_time3_sec
        )
{
    weight_alpha = 0.5f / new_fps;

    left_detector.SetLeftObjectTime0(1 * new_fps);
    left_detector.SetLeftObjectTime1(left_object_time1_sec * new_fps);
    left_detector.SetLeftObjectTime1(left_object_time2_sec * new_fps);
    left_detector.SetLeftObjectTime1(left_object_time3_sec * new_fps);
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::SetLeftObjectTime1
/// \param left_object_time1_
///
void NativeTracker::SetLeftObjectTime1(int left_object_time1_)
{
    left_detector.SetLeftObjectTime1(left_object_time1_);
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::SetLeftObjectTime2
/// \param left_object_time2_
///
void NativeTracker::SetLeftObjectTime2(int left_object_time2_)
{
    left_detector.SetLeftObjectTime2(left_object_time2_);
}

////////////////////////////////////////////////////////////////////////////
void NativeTracker::SetLeftObjectTime3(int left_object_time3_)
{
    left_detector.SetLeftObjectTime3(left_object_time3_);
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::Reset
/// \param hard
///
void NativeTracker::Reset(
        bool hard
        )
{
    if (hard)
    {
        del_objects_count = 0;
    }
    else
    {
        for (objects_container::const_iterator iter = objects_history.begin(); iter != objects_history.end(); ++iter)
        {
            add_uid_to_del_objects((*iter)->uid);
        }
    }

    objects_count = 0;
    objects_history.clear();

    left_detector.Reset();
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::Track
/// \param regions
/// \param videoHeader
/// \param selection_time
/// \param show_trajectory
/// \param zones
/// \param lines
///
void NativeTracker::Track(
        regions_container& regions,
        regions_container& update_regions,
        const VideoHeader& videoHeader,
        int selection_time,
        bool show_trajectory,
        const zones_cont& zones,
        const lines_cont& lines
        )
{
    // Обнуляем количество найденных объектов
    objects_count = 0;
    // Обнуляем количество удалённых объектов
    del_objects_count = 0;

    // Для объектов, найденных на предыдущих кадрах, ищем подходящие регионы
    for (objects_container::iterator iter_obj = objects_history.begin(); iter_obj != objects_history.end();)
    {
#if 1
#if ADV_OUT
        RECT_ r((*iter_obj)->get_rect());
        std::array<uchar, 3> cl = { 0, 0xff, 0 };
        paint_h_line<3>(videoHeader.adv_buf_rgb24, 3 * 2 * videoHeader.frame_width, r.left, r.right, r.top, cl);
        paint_h_line<3>(videoHeader.adv_buf_rgb24, 3 * 2 * videoHeader.frame_width, r.left, r.right + 1, r.bottom, cl);

        paint_v_line<3>(videoHeader.adv_buf_rgb24, 3 * 2 * videoHeader.frame_width, r.left, r.top, r.bottom, cl);
        paint_v_line<3>(videoHeader.adv_buf_rgb24, 3 * 2 * videoHeader.frame_width, r.right, r.top, r.bottom, cl);
#endif
#endif

        // Ищем подходящий для объекта регион с учётом возможного нахождения координат объекта
        regions_container::iterator find_region(find_region_by_center(regions, (*iter_obj)->get_new_center_x(), (*iter_obj)->get_new_center_y(), (*iter_obj)->width(), (*iter_obj)->height()));

        // Если регион не найден,
        if (find_region == regions.end() &&
                (*iter_obj)->get_new_center_x() != (*iter_obj)->get_last_center_x() &&
                (*iter_obj)->get_new_center_y() != (*iter_obj)->get_last_center_y())
        {
            // то повторяем поиск с координатами центра, полученными на предыдущем шаге
            find_region = find_region_by_center(regions, (*iter_obj)->get_last_center_x(), (*iter_obj)->get_last_center_y(), (*iter_obj)->width(), (*iter_obj)->height());
        }
        // Если, наконец, подходящий регион нашёлся, то:
        if (find_region != regions.end())
        {
            // Изменяем параметры объекта:
            // 1. вычисляем новый вес объекта
            (*iter_obj)->weight = (1 - weight_alpha) * (*iter_obj)->weight + weight_alpha;
            // 2. увеличиваем время жизни объекта
            (*iter_obj)->life_time++;
            // 3. запоминаем координаты объекта
            (*iter_obj)->set_rect(find_region->get_left(), find_region->get_right(), find_region->get_top(), find_region->get_bottom());
            // 4. обнуляем количество кадров, на которых объект не был найден
            (*iter_obj)->frames_left = 0;

            // Объект выводится, если он существует достаточно долго
            if ((*iter_obj)->life_time > selection_time)
            {
                // Объект считается принадлежащим к спискам возможно являющихся оставленными предметами
                left_detector.CheckShadyLeftObject(**iter_obj);

                // Если объект двигается
                if (!left_detector.CheckLeftObject(**iter_obj))
                {
                    // Попадает ли объект в зоны детекции
                    mstring zone_name;
                    if (is_in_zone(zones, *find_region, &zone_name))
                    {
                        // Проверяем на пересечение с линиями и обновляем координаты центра объекта и скорость его смещения
                        with_line_intersect(lines, **iter_obj, find_region->get_center_x(), find_region->get_center_y(), videoHeader);

                        // Задаём новые значения центра объекта
                        (*iter_obj)->set_last_center(find_region->get_center_x(), find_region->get_center_y());
                        // Удаляем идентификатор из списка объектов, возможно являющихся оставленными
                        if (!(*iter_obj)->get_left_frames())
                        {
                            left_detector.DelUidFromShadyLeftObjects((*iter_obj)->uid);
                        }
                        else
                        {
                            left_detector.IncTimeShadyLeftObjects((*iter_obj)->uid);
                        }

                        // Отправляем объект на вывод
                        add_object_to_out_rects(*find_region, **iter_obj, unknown_object, zone_name, videoHeader, show_trajectory);
                    }
                    else // Объект не попал в зоны детекции
                    {
                        // Задаём новые значения центра объекта
                        (*iter_obj)->set_last_center(find_region->get_center_x(), find_region->get_center_y());
                        // Удаляем идентификатор из списка объектов, возможно являющихся оставленными
                        if (!(*iter_obj)->get_left_frames())
                        {
                            left_detector.DelUidFromShadyLeftObjects((*iter_obj)->uid);
                        }
                        else
                        {
                            left_detector.IncTimeShadyLeftObjects((*iter_obj)->uid);
                        }
                    }
                }
                else // Объект не двигается - становится оставленным предметом
                {
                    // Попадает ли оставленный предмет в зоны детекции
                    if (is_in_zone(zones, *find_region, nullptr))
                    {
                        // Cоздаём оставленный предмет
                        left_detector.NewLeftObject((*iter_obj)->uid, *find_region);
                    }

                    // Удаляем найденный регион, чтобы избежать совпадений с другими объектами
                    update_regions.push_back(*find_region);
                    regions.erase(find_region);

                    // Удаляем из списка объектов
                    del_object(*iter_obj, true);
                    iter_obj = objects_history.erase(iter_obj);
                    continue;
                }
            }
            else // Объект существует недостаточно долго
            {
                // Задаём новые значения центра объекта
                (*iter_obj)->set_last_center(find_region->get_center_x(), find_region->get_center_y());
                // Удаляем идентификатор из списка объектов, возможно являющихся оставленными
                if (!(*iter_obj)->get_left_frames())
                {
                    left_detector.DelUidFromShadyLeftObjects((*iter_obj)->uid);
                }
                else
                {
                    left_detector.IncTimeShadyLeftObjects((*iter_obj)->uid);
                }
            }

            // Удаляем найденный регион, чтобы избежать совпадений с другими объектами
            regions.erase(find_region);
        }
        else // Если подходящий регион не найден, то:
        {
            // 1. пересчитываем вес объекта
            (*iter_obj)->weight = (1 - weight_alpha) * (*iter_obj)->weight;
            // 2. если вес меньше допустимого, то объект удаляется
            if ((*iter_obj)->weight < weight_threshold)
            {
                del_object(*iter_obj, true);
                iter_obj = objects_history.erase(iter_obj);
                continue;
            }
            // 3. пересчитываем его центр в соответствии с последней скоростью объекта
            (*iter_obj)->recalc_center();
            // 4. Если объект вышел за границы кадра - удаляем
            if (!in_range<int>((*iter_obj)->get_last_center_x(), 0, videoHeader.frame_width) ||
                    !in_range<int>((*iter_obj)->get_last_center_y(), 0, videoHeader.frame_height))
            {
                del_object(*iter_obj, true);
                iter_obj = objects_history.erase(iter_obj);
                continue;
            }
            // 5. Увеличиваем число кадров, на которых объект не был найден
            (*iter_obj)->frames_left++;

#if 1           // Закомментировано, так как выводится очень много левых объектов, можно включать для тестирования
            //6. Отправляем объект на вывод
            if ((*iter_obj)->life_time > selection_time &&
                    (*iter_obj)->frames_left < selection_time / 4)
            {
                //Попадает ли оставленный предмет в зоны детекции
                mstring zone_name;
                if (is_in_zone(zones, **iter_obj, &zone_name))
                {
                    add_object_to_out_rects(**iter_obj, **iter_obj, (*iter_obj)->get_type(), zone_name, videoHeader, show_trajectory);
                }
            }
#endif
        }

        ++iter_obj;
    }

    // Для оставшихся регионов (у которых нет подходящего объекта) создаём новые объекты, но пока не обводим
    for (regions_container::iterator iter_reg = regions.begin(); iter_reg != regions.end(); ++iter_reg)
    {
        objects_history.push_back(std::unique_ptr<CTrackingObject>(new CTrackingObject(iter_reg->get_center_x(), iter_reg->get_center_y(), get_free_uid())));

        (*objects_history.rbegin())->set_rect(iter_reg->get_left(), iter_reg->get_right(), iter_reg->get_top(), iter_reg->get_bottom());
    }
    //Oчищаем список регионов
    regions.clear();

    // Сортируем объекты по времени жизни
    objects_history.sort(CTrackingObject::life_bigger);

    // Удаляем объекты, не обнаруживающиеся в течение некоторого времени
    left_detector.AnalyzeShadyLeftObjects();

    // Анализ оставленных предметов
    left_detector.AnalyzeLeftedObjects(videoHeader);
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::get_free_uid
/// \return
///
unsigned int NativeTracker::get_free_uid()
{
#if 0
    // Получаем простым перебором
    unsigned int ret_val = 1;
    for (; ret_val < std::numeric_limits<unsigned int>::max(); ++ret_val)
    {
        // Поиск среди удалённых объектов
        if (std::find(del_objects.begin(), del_objects.begin() + del_objects_count, ret_val) != del_objects.begin() + del_objects_count)
            continue;

        // Поиск среди существующих
        bool find_uid = false;
        for (objects_container::const_iterator iter = objects_history.begin(); iter != objects_history.end(); ++iter)
        {
            if (ret_val == (*iter)->uid)
            {
                find_uid = true;
                break;
            }
        }
        if (!find_uid)
            break;
    }
    return ret_val;
#else
    // Глобальная индексация
    unsigned int ret_val = lastUid;
    if (lastUid < std::numeric_limits<unsigned int>::max())
    {
        ++lastUid;
    }
    else
    {
        lastUid = 1;
    }
    return ret_val;
#endif
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::add_uid_to_del_objects
/// \param uid
///
void NativeTracker::add_uid_to_del_objects(
        unsigned int uid
        )
{
    // Добавление в список удалённых объектов
    if (++del_objects_count > del_objects.size())
    {
        del_objects.push_back(uid);
    }
    else
    {
        del_objects[del_objects_count - 1] = uid;
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::del_object
/// \param object
/// \param del_adv_data
///
void NativeTracker::del_object(
        std::unique_ptr<CTrackingObject>& object,
        bool del_adv_data
        )
{
    if (del_adv_data)
    {
        // Удаление из списка возможно оставленных предметов
        left_detector.DelFromShadyLeftObjects(object->uid);
    }
    add_uid_to_del_objects(object->uid);
    object = nullptr;
}

////////////////////////////////////////////////////////////////////////////
objects_container::iterator NativeTracker::get_object_by_region(
        const CObjectRegion& region,
        objects_container::iterator from_obj
        )
{
    objects_container::iterator find_object(objects_history.end());
    int min_x(std::numeric_limits<int>::max() / 2 - 1);
    int min_y(std::numeric_limits<int>::max() / 2 - 1);

    int tmp_x, tmp_y;
    int c_x = region.get_center_x(), c_y = region.get_center_y();

    int motion_delta_x = 2 * (*from_obj)->width();
    int motion_delta_y = 2 * (*from_obj)->height();

    // Ищем объект, центр которого наиболее приближен к центру региона
    for (objects_container::iterator iter = ++from_obj; iter != objects_history.end(); ++iter)
    {
        tmp_x = abs((*iter)->get_new_center_x() - c_x);
        tmp_y = abs((*iter)->get_new_center_y() - c_y);

        if ((tmp_x + tmp_y < min_x + min_y) && (tmp_x < motion_delta_x) && (tmp_y < motion_delta_y))
        {
            min_x = tmp_x;
            min_y = tmp_y;
            find_object = iter;
        }
    }
    return find_object;
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::find_region_by_center
/// \param regions
/// \param c_x
/// \param c_y
/// \param width
/// \param height
/// \return
///
regions_container::iterator NativeTracker::find_region_by_center(
        regions_container& regions,
        int c_x,
        int c_y,
        int width,
        int height/*, Hist& object_hist*/
        )
{
    regions_container::iterator find_region(regions.end());
    int min_x(std::numeric_limits<int>::max() / 2 - 1);
    int min_y(std::numeric_limits<int>::max() / 2 - 1);

    width *= 2;
    height *= 2;

#if 0
    RECT_ source_rect = RECT_(c_x, c_x + width, c_y, c_y + height);
    int maxIntersectionArea = 1;
#endif

    // Ищем регион, центр которого наиболее приближен к центру рассматриваемого объекта
    for (regions_container::iterator iter_reg = regions.begin(); iter_reg != regions.end(); ++iter_reg)
    {
#if 0
        int intersectionArea = get_intersect_area<int>(source_rect,
                                                       RECT_(iter_reg->get_left(), iter_reg->get_right(), iter_reg->get_top(), iter_reg->get_bottom()));

        if (intersectionArea > maxIntersectionArea)
        {
            find_region = iter_reg;
            maxIntersectionArea = intersectionArea;
        }
#else
        int tmp_x = abs(iter_reg->get_center_x() - c_x);
        int tmp_y = abs(iter_reg->get_center_y() - c_y);

        if ((tmp_x + tmp_y < min_x + min_y) && (tmp_x < width) && (tmp_y < height))
        {
            min_x = tmp_x;
            min_y = tmp_y;
            find_region = iter_reg;
        }
#endif
    }
    return find_region;
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::GetObjects
/// \param rect_arr
/// \param rect_count
///
void NativeTracker::GetObjects(
        const CObjRect* &rect_arr,
        size_t& rect_count
        ) const
{
    rect_count = objects_count;
    if (rect_count)
    {
        rect_arr = &obj_rects[0];
    }
    else
    {
        rect_arr = nullptr;
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::SetOneObject
/// \param uid
/// \param left
/// \param right
/// \param top
/// \param bottom
///
void NativeTracker::SetOneObject(
        unsigned int uid,
        int left,
        int right,
        int top,
        int bottom
        )
{
    if (uid)
    {
        for (objects_container::const_iterator iter = objects_history.begin(); iter != objects_history.end(); ++iter)
        {
            if (uid != (*iter)->uid)
                add_uid_to_del_objects((*iter)->uid);
        }
        objects_history.clear();
    }
    else
    {
        uid = 1;
        del_objects_count = 0;
    }

    if (!obj_rects.size())
    {
        CObjRect object(0, 0, 0, 0, 1, 0, 0);
        obj_rects.push_back(object);
    }

    obj_rects[0].left = left;
    obj_rects[0].right = right;
    obj_rects[0].top = top;
    obj_rects[0].bottom = bottom;
    obj_rects[0].uid = uid;
    obj_rects[0].type = unknown_object;
    obj_rects[0].zone_name[0] = '\0';
    obj_rects[0].trajectory_size = 1;
    objects_count = 1;

    left_detector.Reset();
}

////////////////////////////////////////////////////////////////////////////
void NativeTracker::GetDelObjects(
        unsigned int* &uids_arr,
        size_t& uids_count
        )
{
    uids_count = del_objects_count;
    if (uids_count)
    {
        uids_arr = &del_objects[0];
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::add_object_to_out_rects
/// \param rect
/// \param object
/// \param obj_type
/// \param zone_name
/// \param videoHeader
/// \param show_trajectory
///
template<class T>
void NativeTracker::add_object_to_out_rects(
        const T& rect,
        const CTrackingObject& object,
        object_types obj_type,
        const mstring& zone_name,
        const VideoHeader& videoHeader,
        bool show_trajectory
        )
{
    int obj_left = rect.get_left();
    int obj_right = rect.get_right();
    int obj_top = rect.get_top();
    int obj_bottom = rect.get_bottom();
    if (videoHeader.analyze_area.left)
    {
        obj_left += videoHeader.left_padding;
        obj_right += videoHeader.left_padding;
    }
    if (videoHeader.analyze_area.top)
    {
        obj_top += videoHeader.top_padding;
        obj_bottom += videoHeader.top_padding;
    }

    // Здесь сделана попытка избежать лишних выделений памяти: массив obj_rects никогда не уменьшается
    // Он может только увеличивать свой размер, если на последующих кадрах выделенных объектов больше, чем на предыдущих
    if (++objects_count > obj_rects.size())
    {
        obj_rects.push_back(CObjRect(obj_left, obj_right, obj_top, obj_bottom, object.uid, object.get_new_center_x(), object.get_new_center_y()));
        if (show_trajectory)
        {
            object.get_trajectory(*obj_rects.rbegin(), videoHeader.frame_width, videoHeader.frame_height, videoHeader.left_padding, videoHeader.top_padding);
        }
        obj_rects.rbegin()->type = obj_type;
        obj_rects.rbegin()->zone_name = zone_name;
    }
    else
    {
        obj_rects[objects_count - 1].left = obj_left;
        obj_rects[objects_count - 1].right = obj_right;
        obj_rects[objects_count - 1].top = obj_top;
        obj_rects[objects_count - 1].bottom = obj_bottom;
        obj_rects[objects_count - 1].uid = object.uid;
        obj_rects[objects_count - 1].new_center_x = videoHeader.left_padding + object.get_x_future_val(videoHeader.fps);
        obj_rects[objects_count - 1].new_center_y = videoHeader.top_padding + object.get_y_future_val(videoHeader.fps);
        if (show_trajectory)
        {
            object.get_trajectory(obj_rects[objects_count - 1], videoHeader.frame_width, videoHeader.frame_height, videoHeader.left_padding, videoHeader.top_padding);
        }
        else
        {
            obj_rects[objects_count - 1].trajectory_size = 1;
        }

        obj_rects[objects_count - 1].type = obj_type;

        obj_rects[objects_count - 1].zone_name = zone_name;
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::GetObject
/// \param obj_ind
/// \param objRect
/// \return
///
bool NativeTracker::GetObject(
        size_t obj_ind,
        CObjRect& objRect
        ) const
{
    if (obj_ind >= obj_rects.size())
    {
        return false;
    }
    else
    {
        objRect = obj_rects[obj_ind];

        return true;
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::GetLeftObjects
/// \param rect_arr
/// \param rect_count
///
void NativeTracker::GetLeftObjects(
        const CLeftObjRect* &rect_arr,
        size_t& rect_count
        ) const
{
    left_detector.GetLeftObjects(rect_arr, rect_count);
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::is_in_zone
/// \param zones
/// \param rect
/// \param zone_name
/// \return
///
template<typename ZONES_T, typename T>
bool NativeTracker::is_in_zone(
        const ZONES_T& zones,
        const T& rect,
        mstring* zone_name
        ) const
{
    if (zone_name)
    {
        *zone_name = "";
    }
    if (zones.empty())
    {
        return true;
    }

    for (const auto& zone : zones)
    {
        if (zone.use_detection &&
                segments_superposition(zone.left, zone.right, rect.get_left(), rect.get_right()) &&
                segments_superposition(zone.top, zone.bottom, rect.get_top(), rect.get_bottom()))
        {
            if (zone_name)
            {
                *zone_name = zone.name.c_str();
            }
            return true;
        }
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::with_line_intersect
/// \param lines
/// \param obj
/// \param new_center_x
/// \param new_center_y
/// \param videoHeader
/// \return
///
bool NativeTracker::with_line_intersect(
        const lines_cont& lines,
        const CTrackingObject &obj,
        int new_center_x,
        int new_center_y,
        const VideoHeader& videoHeader
        )
{
    bool ret_val(false);

    const float_t r = 50000.0;

    for (size_t i = 0; i < lines.size(); ++i)
    {
        // Проверка пересечения последнего шага траектории движения объекта и пользовательской линии

        const float_t lx1 = wnd_to_x(lines[i].x1, videoHeader.frame_width, -r, r);
        const float_t lx2 = wnd_to_x(lines[i].x2, videoHeader.frame_width, -r, r);
        const float_t ly1 = wnd_to_x(lines[i].y1, videoHeader.frame_height, -r, r);
        const float_t ly2 = wnd_to_x(lines[i].y2, videoHeader.frame_height, -r, r);

        const float_t objx1 = wnd_to_x(obj.get_last_center_x(), videoHeader.frame_width, -r, r);
        const float_t objy1 = wnd_to_x(obj.get_last_center_y(), videoHeader.frame_height, -r, r);
        const float_t objx2 = wnd_to_x(new_center_x, videoHeader.frame_width, -r, r);
        const float_t objy2 = wnd_to_x(new_center_y, videoHeader.frame_height, -r, r);

        if (is_intersect(lx1, ly1, lx2, ly2, objx1, objy1, objx2, objy2))
        {
#if 0
            // Необходимо узнать с какой стороны произошло пересечение
            // Для этого выберается точка, заведомо лежащая на стороне 1
            float_t x(0), y(0);
            if ((lines[i].x1 <= lines[i].x2) && (lines[i].y1 > lines[i].y2))
            {
                x = wnd_to_x((lines[i].x1 + lines[i].x2) / 2 - 10, videoHeader.frame_width, -r, r);
                y = wnd_to_x((lines[i].y1 + lines[i].y2) / 2 - 10, videoHeader.frame_height, -r, r);
            }
            else
            {
                if ((lines[i].x1 <= lines[i].x2) && (lines[i].y1 <= lines[i].y2))
                {
                    x = wnd_to_x((lines[i].x1 + lines[i].x2) / 2 + 10, videoHeader.frame_width, -r, r);
                    y = wnd_to_x((lines[i].y1 + lines[i].y2) / 2 - 10, videoHeader.frame_height, -r, r);
                }
                else
                {
                    if ((lines[i].x1 > lines[i].x2) && (lines[i].y1 > lines[i].y2))
                    {
                        x = wnd_to_x((lines[i].x1 + lines[i].x2) / 2 - 10, videoHeader.frame_width, -r, r);
                        y = wnd_to_x((lines[i].y1 + lines[i].y2) / 2 + 10, videoHeader.frame_height, -r, r);
                    }
                    else
                    {
                        if ((lines[i].x1 > lines[i].x2) && (lines[i].y1 <= lines[i].y2))
                        {
                            x = wnd_to_x((lines[i].x1 + lines[i].x2) / 2 + 10, videoHeader.frame_width, -r, r);
                            y = wnd_to_x((lines[i].y1 + lines[i].y2) / 2 + 10, videoHeader.frame_height, -r, r);
                        }
                    }
                }
            }
            direction = is_intersect(lx1, ly1, lx2, ly2, objx1, objy1, x, y) ? 1 : 0;
#endif
            ret_val = true;
        }
    }
    return ret_val;
}

} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
