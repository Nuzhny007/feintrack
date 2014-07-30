#include <fstream>
#include "feintrack_objects.h"
////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
////////////////////////////////////////////////////////////////////////////
CObjectRegion::CObjectRegion()
    : obj_pxl_count(0)
{
}
////////////////////////////////////////////////////////////////////////////
CObjectRegion::CObjectRegion(int left_, int right_, int top_, int bottom_, size_t obj_pxl_count_)
    : left(left_), right(right_), top(top_), bottom(bottom_), obj_pxl_count(obj_pxl_count_)
{
}
////////////////////////////////////////////////////////////////////////////
CObjectRegion::CObjectRegion(const CObjectRegion& region)
    : left(region.left), right(region.right), top(region.top), bottom(region.bottom), obj_pxl_count(region.obj_pxl_count)
{
}
////////////////////////////////////////////////////////////////////////////
CObjectRegion::~CObjectRegion()
{
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::set_left(int left_)
{
    left = left_;
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::set_right(int right_)
{
    right = right_;
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::set_top(int top_)
{
    top = top_;
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::set_bottom(int bottom_)
{
    bottom = bottom_;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::get_left() const
{
    return left;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::get_right() const
{
    return right;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::get_top() const
{
    return top;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::get_bottom() const
{
    return bottom;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::width() const
{
    return right - left + 1;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::height() const
{
    return bottom - top + 1;
}
////////////////////////////////////////////////////////////////////////////
bool CObjectRegion::in_region(int x, int y) const
{
    return ((x > left - region_h_granz) && (x <= right + region_h_granz)) &&
            ((y > top - region_v_granz) && (y <= bottom + region_v_granz));
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::add_point(int x, int y)
{
    //Необходима ещё проверка на (x > left - region_granz) и (x < right + region_granz),
    //но мы её опускаем, т.к. данная функция вызывается только после удачного вызова in_region,
    //в котором эта проверка осуществляется.
    //Аналогично и для (y > top - region_granz) и (y < bottom + region_granz)
    if (x < left)
        left = x;
    else
        if (x > right)
            right = x;

    if (y < top)
        top = y;
    else
        if (y > bottom)
            bottom = y;

    ++obj_pxl_count;
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::regions_merging(const CObjectRegion& region)
{
    if (left > region.left)
        left = region.left;
    if (right < region.right)
        right = region.right;

    if (top > region.top)
        top = region.top;
    if (bottom < region.bottom)
        bottom = region.bottom;

    obj_pxl_count += region.obj_pxl_count;
}
////////////////////////////////////////////////////////////////////////////
bool CObjectRegion::near_region(int left_, int top_) const
{
    return in_range(left_, left, right) && in_range(top_, top, bottom);
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::add_rect(int right_, int bottom_, size_t obj_pxl_count_)
{
    if (right < right_)
        right = right_;
    if (bottom < bottom_)
        bottom = bottom_;
    obj_pxl_count += obj_pxl_count_;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::get_center_x() const
{
    return (right + left) / 2;
}
////////////////////////////////////////////////////////////////////////////
int CObjectRegion::get_center_y() const
{
    return (bottom + top) / 2;
}
////////////////////////////////////////////////////////////////////////////
float_t CObjectRegion::density() const
{
    return (float_t)obj_pxl_count / (float_t)(width() * height());
}
////////////////////////////////////////////////////////////////////////////
void CObjectRegion::resize_to_max_granz(int max_width, int max_height)
{
    left -= region_h_granz;
    set_range(left, 0, max_width);
    right += region_h_granz;
    set_range(right, 0, max_width);
    top -= region_v_granz;
    set_range(top, 0, max_height);
    bottom += region_v_granz;
    set_range(bottom, 0, max_height);
}
////////////////////////////////////////////////////////////////////////////
bool CObjectRegion::size_bigger(const CObjectRegion& reg1, const CObjectRegion& reg2)
{
    return reg1.width() + reg1.height() > reg2.width() + reg2.height();
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
const float_t CTrackingObject::default_weight = 0.7;
////////////////////////////////////////////////////////////////////////////
CTrackingObject::CTrackingObject(int center_x_, int center_y_, unsigned int uid_)
    :
      center_x(center_x_),
      center_y(center_y_),
      dx(0),
      dy(0),
      left_center_x(center_x_),
      left_center_y(center_y_),
      frames_in_eps(0),
      left_epsilon(10),
      #if LIN_MNK
      coords_collected(1),
      kx(0.0), bx(0.0), ky(0.0), by(0.0),
      #endif
      obj_recogn_count(0),
      type(unknown_object),
      merge_object1(nullptr),
      merge_object2(nullptr),
      uid(uid_),
      weight(default_weight),
      life_time(0),
      frames_left(0)
{
#if LIN_MNK
    stat[0] = POINT_<int>(center_x, center_y);
#endif
    traectory_x.push_back(traectory_cont::value_type(0, center_x));
    traectory_y.push_back(traectory_cont::value_type(0, center_y));
}
////////////////////////////////////////////////////////////////////////////
CTrackingObject::CTrackingObject(const CTrackingObject &obj)
    :
      center_x(obj.center_x),
      center_y(obj.center_y),
      dx(obj.dx),
      dy(obj.dy),
      left_center_x(obj.left_center_x),
      left_center_y(obj.left_center_y),
      frames_in_eps(obj.frames_in_eps),
      left_epsilon(obj.left_epsilon),
      #if LIN_MNK
      coords_collected(obj.coords_collected),
      kx(obj.kx),
      bx(obj.bx),
      ky(obj.ky),
      by(obj.by),
      stat(obj.stat),
      #endif
      obj_recogn_count(obj.obj_recogn_count),
      type(obj.type),
      uid(obj.uid),
      weight(obj.weight),
      life_time(obj.life_time),
      frames_left(obj.frames_left)
{
    merge_object1 = obj.merge_object1;
    merge_object2 = obj.merge_object2;
    traectory_x.assign(obj.traectory_x.begin(), obj.traectory_x.end());
    traectory_y.assign(obj.traectory_y.begin(), obj.traectory_y.end());
}
////////////////////////////////////////////////////////////////////////////
CTrackingObject::~CTrackingObject()
{
    delete merge_object1;
    delete merge_object2;
}
////////////////////////////////////////////////////////////////////////////
object_types CTrackingObject::get_type() const
{
#if 1
    return type;
#else
    if (obj_recogn_count > min_recogn_count)
        return type;
    else
        return unknown;
#endif
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::set_new_type(object_types new_type)
{
    if (new_type == type)
    {
        if (new_type != unknown_object)
            ++obj_recogn_count;
        else
            obj_recogn_count = 0;
    }
    else
    {
        if (new_type != unknown_object)
        {
            if (type != unknown_object)
                obj_recogn_count = std::max(0, obj_recogn_count - 1);

            if (!obj_recogn_count)
                type = new_type;
        }
    }
}
////////////////////////////////////////////////////////////////////////////
bool CTrackingObject::have_merge_object() const
{
    return merge_object1 && merge_object2;
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::add_merge_obj(const CTrackingObject &merge_object)
{
    merge_object1 = new CTrackingObject(*this);
    merge_object2 = new CTrackingObject(merge_object);

    if (merge_object.weight > weight)
        weight = merge_object.weight;
    if (merge_object.life_time > life_time)
        life_time = merge_object.life_time;
}
////////////////////////////////////////////////////////////////////////////
CTrackingObject *CTrackingObject::get_merge_object(size_t ind)
{
    switch (ind)
    {
    case 1:
        return merge_object1;
    case 2:
        return merge_object2;
    default:
        return nullptr;
    }
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::inc_merge_frames()
{
    merge_object1->life_time++;
    merge_object1->recalc_center();
    merge_object2->life_time++;
    merge_object2->recalc_center();
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::set_merge_objects_to_null()
{
    merge_object1 = nullptr;
    merge_object2 = nullptr;
}
////////////////////////////////////////////////////////////////////////////
bool CTrackingObject::has_merge_object(unsigned int object_uid) const
{
    return (merge_object1 && (merge_object1->uid == object_uid)) || (merge_object2 && (merge_object2->uid == object_uid));
}
////////////////////////////////////////////////////////////////////////////
#if !LIN_MNK
int CTrackingObject::predict(const traectory_cont& traectory, const traectory_cont& orig_traectory, int delta_time) const
{
    if (orig_traectory.size() == 1)
        return orig_traectory[0].y;

    traectory_cont::value_type p1 = traectory[traectory.size() - 2];
    traectory_cont::value_type p2 = traectory[traectory.size() - 1];

    bool use_dp_predict = true;

    // Проверка величины текущей и предыдущей длин участков траектории для определения типа предсказания
    if (traectory.size() > 2)
    {
        traectory_cont::value_type p0 = traectory[traectory.size() - 3];
        // Если текущий участок достаточно короток
        if (p2.x - p1.x < 4 * dp_epsilon)
        {
            // Также короток и предыдущий участок
            if (p1.x - p0.x < 4 * dp_epsilon)
            {
                // коротка и сумма двух участков
                if (p2.x - p0.x < 4 * dp_epsilon)
                {
                    // то проверям длину и предпредыдущего участка
                    if (traectory.size() > 3)
                    {
                        traectory_cont::value_type p_1 = traectory[traectory.size() - 4];
                        // Если и он короток, то используем линейное предсказание адаптивным методом наименьших квадратов
                        if (p0.x - p_1.x < 4 * dp_epsilon)
                        {
                            use_dp_predict = false;
                        }
                        else // Если предпредыдущий участок достаточно велик, то предсказываем по нему
                        {
                            p2 = p0;
                            p1 = p_1;
                        }
                    }
                    else // Если предпредыдущего участка нет, то используем линейное предсказание адаптивным методом наименьших квадратов
                    {
                        use_dp_predict = false;
                    }
                }
                else
                {
                    // Если текущий и предыдущий участки коротки, но сумма их достаточно велика, то используем линейное предсказание адаптивным методом наименьших квадратов
                    use_dp_predict = false;
                }
            }
            else // Если текущий участок короток, а предыдущий достаточно длинный, то предсказываем по нему
            {
                p2 = p1;
                p1 = p0;
            }
        }
    }
    if (use_dp_predict)
    {
        // Предсказание по участку траектории, найденному алгоритмом Дугласа-Пекера
        return (((p2.x + delta_time) - p1.x) * (p2.y - p1.y)) / (p2.x - p1.x) + p1.y;
    }
    else
    {
        // Линейное предсказание адаптивным методом наименьших квадратов по некоторой части траектории объекта
        double x0(0.), v0_x(0.), y0(0.), v0_y(0.);
        get_lin_regress_params_a(orig_traectory, std::max(0, (int)orig_traectory.size() - 25), orig_traectory.size(), v0_x, x0, v0_y, y0);
        return static_cast<int>(y0 + v0_y * (p2.x + delta_time));
    }
}
#endif
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_x_future_val(int future_time) const
{
#if LIN_MNK
    if (coords_collected < STAT_FRAME_COUNT)
        return center_x + (future_time * dx) / 2;
    else
        return static_cast<int>(kx * (future_time + stat.size()) + bx);
#else
    return predict(dp_traectory_x, traectory_x, future_time);
#endif
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_y_future_val(int future_time) const
{
#if LIN_MNK
    if (coords_collected < STAT_FRAME_COUNT)
        return center_y + (future_time * dy) / 2;
    else
        return static_cast<int>(ky * (future_time + stat.size()) + by);
#else
    return predict(dp_traectory_y, traectory_y, future_time);
#endif
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_new_center_x() const
{
    return get_x_future_val(1);
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_new_center_y() const
{
    return get_y_future_val(1);
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_last_center_x() const
{
    return center_x;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_last_center_y() const
{
    return center_y;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_left_frames() const
{
    return frames_in_eps;
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::set_left_frames(int frames_in_eps_)
{
    frames_in_eps = frames_in_eps_;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_left_epsilon() const
{
    return left_epsilon;
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::set_last_center(int new_center_x, int new_center_y)
{
    //Небольшое сглаживание траектории
    new_center_x = (center_x + 2 * new_center_x) / 3;
    new_center_y = (center_y + 2 * new_center_y) / 3;

    if ((abs(left_center_x - new_center_x) > left_epsilon) || (abs(left_center_y - new_center_y) > left_epsilon))
    {
        left_center_x = new_center_x;
        left_center_y = new_center_y;

        frames_in_eps = 0;
    }
    else
    {
        ++frames_in_eps;
    }

    dx = new_center_x - center_x;
    dy = new_center_y - center_y;

    traectory_x.push_back(traectory_cont::value_type(traectory_x.back().x + 1, new_center_x));
    if (traectory_x.size() > MaxTrajectorySize)
    {
        traectory_x.pop_front();
    }

    traectory_y.push_back(traectory_cont::value_type(traectory_y.back().x + 1, new_center_y));
    if (traectory_y.size() > MaxTrajectorySize)
    {
        traectory_y.pop_front();
    }

#if 0
    // Debug case
    if (traectory_x.size() > MaxTrajectorySize / 2)
    {
        char tmp[1024];
        static int iii = 0;
        sprintf(tmp, "/home/nuzhny/Documents/tmp/%05u_%04i_x.txt", traectory_x.size(), iii);
        std::ofstream ff(tmp);
        for (size_t i = 0; i < traectory_x.size(); ++i)
        {
            ff << traectory_x[i].y << std::endl;
        }
        sprintf(tmp, "/home/nuzhny/Documents/tmp/%05u_%04i_y.txt", traectory_y.size(), iii);
        ff.close();
        ff.open(tmp);
        for (size_t i = 0; i < traectory_y.size(); ++i)
        {
            ff << traectory_y[i].y << std::endl;
        }
        ++iii;
    }
#endif

    center_x = new_center_x;
    center_y = new_center_y;

#if LIN_MNK
    //Обновляем список координат объекта для расчёта его траектории
    if (coords_collected < STAT_FRAME_COUNT)
        stat[coords_collected++] = POINT_<int>(center_x, center_y);
    else
        stat.shift_last_elem(POINT_<int>(center_x, center_y));

    //Вычисление коэффициентов уравнения траектории движения объекта
    get_lin_regress_params_a(stat, 0, stat.size(), kx, bx, ky, by);
#else
    // Обработка траектории движения объекта алгоритмом Дугласа-Пекера
    dp_traectory_x.clear();
    DouglasPeucker(traectory_x.begin(), traectory_x.end() - 1, dp_traectory_x, dp_epsilon);

    dp_traectory_y.clear();
    DouglasPeucker(traectory_y.begin(), traectory_y.end() - 1, dp_traectory_y, dp_epsilon);
#endif
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::set_rect(int left, int right, int top, int bottom)
{
    rect.left = left;
    rect.right = right;
    rect.top = top;
    rect.bottom = bottom;

    left_epsilon = std::min(width(), height()) / 4;
}
////////////////////////////////////////////////////////////////////////////
const RECT_ &CTrackingObject::get_rect() const
{
    return rect;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_left() const
{
    return rect.left;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_right() const
{
    return rect.right;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_top() const
{
    return rect.top;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::get_bottom() const
{
    return rect.bottom;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::width() const
{
    return rect.right - rect.left + 1;
}
////////////////////////////////////////////////////////////////////////////
int CTrackingObject::height() const
{
    return rect.bottom - rect.top + 1;
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::recalc_center()
{
    set_last_center(get_new_center_x(), get_new_center_y());
}
////////////////////////////////////////////////////////////////////////////
void CTrackingObject::get_traectory(CObjRect &obj_rect, uint frame_width, uint frame_height, int left_padding, int top_padding) const
{
    size_t i = (traectory_x.size() > CObjRect::MAX_TRAECTORY)? (traectory_x.size() - CObjRect::MAX_TRAECTORY): 1;
    obj_rect.traectory_size = 0;
    float alpha = 0.7;
    for (size_t stop = traectory_x.size(); i < stop; ++i)
    {
        int vx = static_cast<int>(alpha * traectory_x[i].y + (1- alpha) * traectory_x[i - 1].y) + left_padding;
        int vy = static_cast<int>(alpha * traectory_y[i].y + (1- alpha) * traectory_y[i - 1].y) + top_padding;
        set_range<int>(vx, 0, frame_width - 1);
        set_range<int>(vy, 0, frame_height - 1);
        obj_rect.traectory[obj_rect.traectory_size].x = vx;
        obj_rect.traectory[obj_rect.traectory_size].y = vy;

        ++obj_rect.traectory_size;
    }
    obj_rect.traectory_size--;
}
////////////////////////////////////////////////////////////////////////////
bool CTrackingObject::weight_bigger(const CTrackingObject &obj1, const CTrackingObject &obj2)
{
    return obj1.weight > obj2.weight;
}
////////////////////////////////////////////////////////////////////////////
bool CTrackingObject::life_bigger(const std::unique_ptr<CTrackingObject>& obj1, const std::unique_ptr<CTrackingObject>& obj2)
{
    return obj1->life_time > obj2->life_time;
}
////////////////////////////////////////////////////////////////////////////
bool CTrackingObject::weight_life_bigger(const CTrackingObject &obj1, const CTrackingObject &obj2)
{
    return obj1.weight * obj1.life_time > obj2.weight * obj2.life_time;
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
