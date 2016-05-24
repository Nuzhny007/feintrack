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
      left_objects_count(0),
      left_object_time0(0),
      left_object_time1(0),
      left_object_time2(0),
      left_object_time3(0),
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

    left_object_time0 = 1 * new_fps;
    left_object_time1 = left_object_time1_sec * new_fps;
    left_object_time2 = left_object_time2_sec * new_fps;
    left_object_time3 = left_object_time3_sec * new_fps;
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::SetLeftObjectTime1
/// \param left_object_time1_
///
void NativeTracker::SetLeftObjectTime1(int left_object_time1_)
{
    left_object_time1 = left_object_time1_;
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::SetLeftObjectTime2
/// \param left_object_time2_
///
void NativeTracker::SetLeftObjectTime2(int left_object_time2_)
{
    left_object_time2 = left_object_time2_;
}

////////////////////////////////////////////////////////////////////////////
void NativeTracker::SetLeftObjectTime3(int left_object_time3_)
{
    left_object_time3 = left_object_time3_;
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
    left_objects_count = 0;

    objects_history.clear();
    shady_left_objects.clear();
    lefted_objects.clear();
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
                if ((*iter_obj)->get_left_frames() == left_object_time0)
                {
                    add_to_shady_left_objects(**iter_obj);
                }

                // Если объект двигается
                if ((*iter_obj)->get_left_frames() < left_object_time1)
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
                            del_uid_from_shady_left_objects((*iter_obj)->uid);
                        }
                        else
                        {
                            inc_time_shady_left_objects((*iter_obj)->uid);
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
                            del_uid_from_shady_left_objects((*iter_obj)->uid);
                        }
                        else
                        {
                            inc_time_shady_left_objects((*iter_obj)->uid);
                        }
                    }
                }
                else // Объект не двигается - становится оставленным предметом
                {
                    // Попадает ли оставленный предмет в зоны детекции
                    if (is_in_zone(zones, *find_region, nullptr))
                    {
                        // Cоздаём оставленный предмет
                        lefted_objects.push_back(CLeftObjView(left_object_time3 - left_object_time1, find_region->get_left(), find_region->get_right(), find_region->get_top(), find_region->get_bottom(), (*iter_obj)->uid));
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
                    del_uid_from_shady_left_objects((*iter_obj)->uid);
                }
                else
                {
                    inc_time_shady_left_objects((*iter_obj)->uid);
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
    analyze_shady_left_objects();

    // Анализ оставленных предметов
    analyze_lefted_objects(videoHeader);
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
        del_from_shady_left_objects(object->uid);
    }
    add_uid_to_del_objects(object->uid);
    object = nullptr;
}

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::add_to_shady_left_objects
/// \param obj
///
void NativeTracker::add_to_shady_left_objects(
        CTrackingObject &obj
        )
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

////////////////////////////////////////////////////////////////////////////
/// \brief NativeTracker::del_from_shady_left_objects
/// \param obj_uid
///
void NativeTracker::del_from_shady_left_objects(
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
/// \brief NativeTracker::del_uid_from_shady_left_objects
/// \param obj_uid
///
void NativeTracker::del_uid_from_shady_left_objects(
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
/// \brief NativeTracker::inc_time_shady_left_objects
/// \param obj_uid
///
void NativeTracker::inc_time_shady_left_objects(
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
/// \brief NativeTracker::analyze_shady_left_objects
///
void NativeTracker::analyze_shady_left_objects()
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
/// \brief NativeTracker::add_left_object_to_out_rects
/// \param left_obj
/// \param type
/// \param videoHeader
///
void NativeTracker::add_left_object_to_out_rects(
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
/// \brief NativeTracker::analyze_lefted_objects
/// \param videoHeader
///
void NativeTracker::analyze_lefted_objects(
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
/// \brief NativeTracker::GetObjects
/// \param rect_arr
/// \param rect_count
///
void NativeTracker::GetObjects(
        CObjRect* &rect_arr,
        size_t& rect_count
        )
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
    obj_rects[0].traectory_size = 1;
    objects_count = 1;

    left_objects_count = 0;
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
            object.get_traectory(*obj_rects.rbegin(), videoHeader.frame_width, videoHeader.frame_height, videoHeader.left_padding, videoHeader.top_padding);
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
            object.get_traectory(obj_rects[objects_count - 1], videoHeader.frame_width, videoHeader.frame_height, videoHeader.left_padding, videoHeader.top_padding);
        }
        else
        {
            obj_rects[objects_count - 1].traectory_size = 1;
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
        )
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
        CLeftObjRect* &rect_arr,
        size_t& rect_count
        )
{
    rect_count = left_objects_count;
    if (rect_count)
    {
        rect_arr = &left_obj_rects[0];
    }
}

////////////////////////////////////////////////////////////////////////////

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
