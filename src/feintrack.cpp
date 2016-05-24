#include "feintrack.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
////////////////////////////////////////////////////////////////////////////

const float_t CFeinTrack::density_threshold = 0.1f;
////////////////////////////////////////////////////////////////////////////

CFeinTrack::CFeinTrack()
    :
      use_square_segmentation(true),
      left_object_time1_sec(15),
      left_object_time2_sec(1 * 60),
      left_object_time3_sec(2 * 60),
      frame_width(0),
      frame_height(0),
      curr_color_type(buf_rgb24),
      fps(25),
      selection_time(25 / 2),
      curr_frame(0),
      cut_shadows(false),
      use_cuda(false),
      cuda_device_ind(0),
      pixel_size(3),
      use_morphology(true),
      min_region_width(5),
      min_region_height(5),
      left_padding(0),
      top_padding(0),
      show_objects(false),
      show_trajectory(false),
      show_left_objects(true),
      use_recognition(false),
      bgrnd_type(norm_back),
      back_substractor(nullptr)
{
    set_bgrnd_type(bgrnd_type);

    selection_time = fps / 2;
    set_fps(fps);

    analyze_area = RECT_(0, 100, 0, 100);

    back_substractor->set_use_cuda(use_cuda);
}
////////////////////////////////////////////////////////////////////////////

CFeinTrack::~CFeinTrack()
{
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_bgrnd_type(bgrnd_substr_types bgrnd_type_)
{
    int sens_level = 60;
    bool detect_patches_of_sunlight = false;

    if (bgrnd_type != bgrnd_type_)
    {
        if (back_substractor)
        {
            sens_level = back_substractor->get_sensitivity();
            detect_patches_of_sunlight = back_substractor->get_detect_patches_of_sunlight();
        }
    }

    bgrnd_type = bgrnd_type_;

    switch (bgrnd_type)
    {
    case norm_back:
        back_substractor = std::unique_ptr<CBackSubstraction>(new CNormBackSubstraction());
        break;

    case gaussian_mixture:
        back_substractor = std::unique_ptr<CBackSubstraction>(new CGaussianMixtureBackSubstr());
        break;
    }

    back_substractor->set_use_cuda(use_cuda);
    back_substractor->set_fps(fps);
    back_substractor->set_sensitivity(sens_level);
    back_substractor->set_detect_patches_of_sunlight(detect_patches_of_sunlight);
}
////////////////////////////////////////////////////////////////////////////

bgrnd_substr_types CFeinTrack::get_bgrnd_type() const
{
    return bgrnd_type;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_use_cuda(bool use_cuda_, int cuda_device_ind_)
{
    cuda_device_ind = cuda_device_ind_;
    if (use_cuda_)
    {
#ifdef USE_GPU
        // Получение числа устройств, потенциально поддерживающих CUDA
        int count = 0;
        bool device_was_founded = false;
        if ((cudaGetDeviceCount(&count) == cudaSuccess) && count)
        {
            // Проход по всем устройствам в поиске подходящего
            cudaDeviceProp prop;
            int cuda_device_counter = -1;     // Подсчёт числа устройств, поддерживающих CUDA
            int correct_cuda_device_ind = -1; // Индекс любого устройства, поддерживающего CUDA
            for (int i = 0; i < count; ++i)
            {
                // Проверка версии устройства. Сейчас поддерживаются все версии
                // 9999 - левое устройство, само число получено опытным путём
                if (cudaGetDeviceProperties(&prop, 0) == cudaSuccess &&
                        prop.major != 9999 &&
                        prop.minor != 9999)
                {
                    correct_cuda_device_ind = i;
                    if (cuda_device_counter + 1 == cuda_device_ind)
                    {
                        cudaSetDevice(i);
                        use_cuda = true;
                        if (back_substractor)
                            back_substractor->set_use_cuda(use_cuda);
                        if (show_objects)
                        {
                            set_show_objects(false);
                            set_show_objects(true);
                        }
                        device_was_founded = true;
                        ++cuda_device_counter;
                        break;
                    }
                    ++cuda_device_counter;
                }
            }
            // Выбранное устройство не найдено - назначаем первое попавшееся
            if (!device_was_founded && correct_cuda_device_ind != -1)
            {
                cudaSetDevice(correct_cuda_device_ind);
                use_cuda = true;
                if (back_substractor)
                    back_substractor->set_use_cuda(use_cuda);
                if (show_objects)
                {
                    set_show_objects(false);
                    set_show_objects(true);
                }
                device_was_founded = true;
            }
        }
        // Подходящее устройство не найдено
        if (!device_was_founded)
        {
            use_cuda = false;
            if (back_substractor)
                back_substractor->set_use_cuda(use_cuda);
        }
#else
        assert(false);
#endif
    }
    else
    {
        use_cuda = false;
        if (back_substractor)
        {
            back_substractor->set_use_cuda(use_cuda);
        }
        if (show_objects)
        {
            set_show_objects(false);
            set_show_objects(true);
        }
    }
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_use_cuda() const
{
    return use_cuda;
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_cuda_device() const
{
    return cuda_device_ind;
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_use_morphology() const
{
    return use_morphology;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_use_morphology(bool use_morphology_)
{
    use_morphology = use_morphology_;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_show_left_objects(bool show_left_objects_)
{
    show_left_objects = show_left_objects_;
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_show_left_objects() const
{
    return show_left_objects;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_show_trajectory(bool show_trajectory_)
{
    show_trajectory = show_trajectory_;
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_show_trajectory() const
{
    return show_trajectory;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_min_region_width(int min_region_width_)
{
    min_region_width = min_region_width_;
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_min_region_width() const
{
    return min_region_width;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_selection_time(int selection_time_)
{
    selection_time = selection_time_;
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_selection_time() const
{
    return selection_time;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_min_region_height(int min_region_height_)
{
    min_region_height = min_region_height_;
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_min_region_height() const
{
    return min_region_height;
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_detect_patches_of_sunlight() const
{
    if (back_substractor)
        return back_substractor->get_detect_patches_of_sunlight();
    else
        return true;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_)
{
    if (back_substractor)
        back_substractor->set_detect_patches_of_sunlight(detect_patches_of_sunlight_);
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_cut_shadows() const
{
    return cut_shadows;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_cut_shadows(bool cut_shadows_)
{
    cut_shadows = cut_shadows_;
}
////////////////////////////////////////////////////////////////////////////

RECT_ CFeinTrack::get_analyze_area() const
{
    return analyze_area;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_analyze_area(const RECT_ &analyze_area_)
{
    if (analyze_area == analyze_area_)
        return;

    analyze_area = analyze_area_;
    if (show_objects)
    {
        set_show_objects(false);
        set_show_objects(true);
    }
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_sensitivity(int sens_level)
{
    if (back_substractor)
        back_substractor->set_sensitivity(sens_level);
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_sensitivity() const
{
    if (back_substractor)
        return back_substractor->get_sensitivity();
    else
        return 60;
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_left_object_time1_sec() const
{
    return left_object_time1_sec;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_left_object_time1_sec(int left_object_time1_sec_)
{
    left_object_time1_sec = left_object_time1_sec_;
    tracker.SetLeftObjectTime1(left_object_time1_sec * fps);
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_left_object_time2_sec() const
{
    return left_object_time2_sec;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_left_object_time2_sec(int left_object_time2_sec_)
{
    left_object_time2_sec = left_object_time2_sec_;
    tracker.SetLeftObjectTime2(left_object_time2_sec * fps);
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_left_object_time3_sec() const
{
    return left_object_time3_sec;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_left_object_time3_sec(int left_object_time3_sec_)
{
    left_object_time3_sec = left_object_time3_sec_;
    tracker.SetLeftObjectTime3(left_object_time3_sec * fps);
}
////////////////////////////////////////////////////////////////////////////

int CFeinTrack::get_fps() const
{
    return fps;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_fps(int new_fps)
{
    fps = new_fps;

    if (back_substractor)
    {
        back_substractor->set_fps(fps);
    }

    tracker.SetFps(fps, left_object_time1_sec, left_object_time2_sec, left_object_time3_sec);
}
////////////////////////////////////////////////////////////////////////////

#if !ADV_OUT
int CFeinTrack::new_frame(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, color_type buf_type)
#else
int CFeinTrack::new_frame(const uchar* buf, uint32_t pitch, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24)
#endif
{
    // Если объекты отображать не нужно, то анализ не проводим
    if (!show_objects)
        return 0;

    pixel_size = get_pixel_size<int>(buf_type);

    // Коррекция размеров кадра, связанная с размером области анализа
    if (!use_cuda)
    {
        left_padding = (analyze_area.left * width) / 100;
        top_padding = (analyze_area.top * height) / 100;
        recalc_correct_zones();
        recalc_correct_lines();

        buf += top_padding * pitch + pixel_size * left_padding;
#if ADV_OUT
        adv_buf_rgb24 += top_padding * 3 * 2 * width + 3 * left_padding;
#endif
        width = ((analyze_area.right - analyze_area.left) * width) / 100;
        height = ((analyze_area.bottom - analyze_area.top) * height) / 100;
    }
    else
    {
        left_padding = 0;
        top_padding = 0;
    }

    // Проверка на смену разрешения, была ли инициализация и т.п.
    bool tmp_use_cuda = use_cuda;
    if (back_substractor->init(width, height, buf_type, use_cuda))
    {
        frame_width = width;
        frame_height = height;
        curr_color_type = buf_type;

        segmentator.init(width, height, use_cuda);
        if (tmp_use_cuda != use_cuda)
        {
            back_substractor->init(width, height, buf_type, use_cuda);
        }

        curr_frame = 0;

        tracker.Reset(true);

        // Пропуск первого кадра при смене разрешения
        return 0;
    }

    // Вычитание фона
#if !ADV_OUT
#ifdef USE_GPU
    if (!back_substractor->background_substraction(curr_frame, buf, pitch, segmentator.get_mask(), segmentator.get_device_mask()))
#else
    if (!back_substractor->background_substraction(curr_frame, buf, pitch, segmentator.get_mask()))
#endif
#else
#ifdef USE_GPU
    if (!back_substractor->background_substraction(curr_frame, buf, pitch, segmentator.get_mask(), segmentator.get_device_mask(), adv_buf_rgb24))
#else
    if (!back_substractor->background_substraction(curr_frame, buf, pitch, segmentator.get_mask(), adv_buf_rgb24))
#endif

#endif
    {
        return 0;
    }

    // Операция математической морфологии "открытие" для фильтрации шума типа качающихся деревьев
    if (use_morphology)
    {
        segmentator.morphology_open(use_cuda);
    }

#if ADV_OUT
    //segmentator.draw_mask(use_cuda, adv_buf_rgb24);
#endif

    // Если используется cuda и требуется распознавание, то копируем маску из видеопамяти в системную
    if (use_recognition && use_cuda)
    {
#ifdef USE_GPU
        segmentator.copy_gpu2cpu();
#else
        assert(false);
#endif
    }

    // Cегментация объектов переднего плана
    regions.clear();
    if (use_cuda)
    {
#ifdef USE_GPU
        segmentator.cuda_segmentation(regions);
#else
        assert(false);
#endif
    }
    else
    {
        if (use_square_segmentation)
        {
            segmentator.square_segmentation(regions);
        }
        else
        {
            segmentator.iterative_segmentation(regions);
        }
    }

    // Предварительный анализ и обработка регионов
    regions_preprocessing(buf, pitch);

    // Создание и анализ поведения объектов на основании найденных на текущем кадре регионов
    VideoHeader videoHeader;
    videoHeader.analyze_area = analyze_area;
    videoHeader.buf = buf;
    videoHeader.fps = fps;
    videoHeader.frame_height = frame_height;
    videoHeader.frame_width = frame_width;
    videoHeader.left_padding = left_padding;
    videoHeader.pitch = pitch;
    videoHeader.pixel_size = pixel_size;
    videoHeader.top_padding = top_padding;
#if ADV_OUT
    videoHeader.adv_buf_rgb24 = adv_buf_rgb24;
#endif
    regions_container update_regions;
    tracker.Track(regions, update_regions, videoHeader, selection_time, show_trajectory, correct_zones, correct_lines);

    for (auto& region : update_regions)
    {
        // Расширяем регион, чтобы скомпенсировать возможное движение
        region.resize_to_max_granz(frame_width - 1, frame_height - 1);
        // И обнуляем статистику внутри него
        back_substractor->reset_statistic_in_region(buf, pitch, region);
    }

    if (++curr_frame == fps)
    {
        curr_frame = 0;
    }

    return 1;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::regions_preprocessing(const uchar* buf, uint32_t pitch)
{
    // Очень маленькие регионы и регионы с маленькой плотностью сразу удаляем
    bool need_update = (curr_frame % fps == fps - 1);
    for (regions_container::iterator it_r = regions.begin(); it_r != regions.end();)
    {
        int min_width = min_region_width;
        int min_height = min_region_height;
        const CZone* zone = get_zone(*it_r);
        if (zone)
        {
            min_width = zone->min_obj_width;
            min_height = zone->min_obj_height;
        }

        if ((it_r->width() < min_width) ||
                (it_r->height() < min_height) ||
                (it_r->density() < density_threshold))
        {
            if (need_update)
            {
                back_substractor->update_statistic_in_region(buf, pitch, *it_r);
            }

            it_r = regions.erase(it_r);
        }
        else
        {
            // Отсечение тени в случае необходимости
            if (cut_shadows)
            {
                cut_shadow(*it_r);
            }
            ++it_r;
        }
    }
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_lines_list(const lines_cont& lines_)
{
    lines = lines_;
    recalc_correct_lines();
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::get_lines_list(lines_cont& lines_) const
{
    lines_ = lines;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::recalc_correct_lines()
{
    correct_lines = lines;
    if (use_cuda)
    {
        return;
    }
    for (size_t i = 0; i < correct_lines.size(); ++i)
    {
        correct_lines[i].x1 -= left_padding;
        correct_lines[i].x2 -= left_padding;
        correct_lines[i].y1 -= top_padding;
        correct_lines[i].y2 -= top_padding;
    }
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_zones_list(const zones_cont& zones_)
{
    zones = zones_;
    recalc_correct_zones();
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::get_zones_list(zones_cont& zones_) const
{
    zones_ = zones;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::recalc_correct_zones()
{
    correct_zones = zones;
    if (use_cuda)
    {
        return;
    }
    for (size_t i = 0; i < correct_zones.size(); ++i)
    {
        correct_zones[i].left -= left_padding;
        correct_zones[i].right -= left_padding;
        correct_zones[i].top -= top_padding;
        correct_zones[i].bottom -= top_padding;
    }
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::get_objects(CObjRect* &rect_arr, size_t& rect_count)
{
    if (show_objects)
    {
        tracker.GetObjects(rect_arr, rect_count);
    }
    else
    {
        rect_count = 0;
        rect_arr = nullptr;
    }
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_one_object(unsigned int uid, int left, int right, int top, int bottom)
{
    tracker.SetOneObject(uid, left, right, top, bottom);
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::get_del_objects(unsigned int* &uids_arr, size_t& uids_count)
{
    if (show_objects)
    {
        tracker.GetDelObjects(uids_arr, uids_count);
    }
    else
    {
        uids_count = 0;
    }
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_object_points(size_t obj_ind, POINTF* points, size_t& max_points)
{
    CObjRect objRect;

    if (use_cuda || !tracker.GetObject(obj_ind, objRect))
    {
        return false;
    }

    uint32_t left = objRect.left - left_padding;
    uint32_t right = objRect.right - left_padding;
    uint32_t top = objRect.top - top_padding;
    uint32_t bottom = objRect.bottom - top_padding;

    mask_type *par = nullptr;
    uint32_t step_x = std::max<uint32_t>(2, (right - left + 1) / 16);
    uint32_t step_y = std::max<uint32_t>(2, (bottom - top + 1) / 16);
    size_t point_ind = 0;
    for (uint32_t y = top + step_y / 2; y < bottom; y += step_y)
    {
        par = &segmentator.get_mask()[(left + step_x / 2) + frame_width * y];
        for (uint32_t x = left + step_x / 2; x < right; x += step_x)
        {
            if (*par)
            {
                if (point_ind >= max_points)
                    break;
                points[point_ind] = POINTF((float)(x + left_padding), (float)(y + top_padding));
                ++point_ind;
            }
            par += step_x;
        }
    }
    max_points = point_ind;
    return true;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::get_left_objects(CLeftObjRect* &rect_arr, size_t& rect_count)
{
    if (show_objects && show_left_objects)
    {
        tracker.GetLeftObjects(rect_arr, rect_count);
    }
    else
    {
        rect_count = 0;
        rect_arr = nullptr;
    }

}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_use_square_segmentation() const
{
    return use_square_segmentation;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_use_square_segmentation(bool use_square_segmentation_)
{
    use_square_segmentation = use_square_segmentation_;
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_use_recognition() const
{
    return use_recognition;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_use_recognition(bool new_val)
{
    use_recognition = new_val;
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::get_show_objects() const
{
    return show_objects;
}
////////////////////////////////////////////////////////////////////////////

void CFeinTrack::set_show_objects(bool new_val)
{
    show_objects = new_val;

    // Если отображение объектов не нужно, то полностью отключаем анализ и удаляем статистику
    if (!show_objects)
    {
        frame_width = 0;
        frame_height = 0;

        back_substractor->set_show_objects(show_objects);

        segmentator.set_show_objects(show_objects);

        tracker.Reset(false);

        curr_frame = 0;
    }
}
////////////////////////////////////////////////////////////////////////////

bool CFeinTrack::cut_shadow(CObjectRegion& region)
{
    // Гистограмма по строкам
    hist_cont horz_hist(region.width(), 0.);

    // Постоение гистограмы для верхней половины региона
    build_horz_hist(region, 0, region.height() / 2, horz_hist, frame_width, &segmentator.get_mask()[0]);

    double area1;
    double center1 = calc_center_mass(horz_hist, area1);

    // Постоение гистограмы для нижней половины региона
    fill(horz_hist.begin(), horz_hist.end(), 0);
    build_horz_hist(region, region.height() / 2, region.height(), horz_hist, frame_width, &segmentator.get_mask()[0]);

    double area2;
    double center2 = calc_center_mass(horz_hist, area2);

    // Проверка взаимного расположения центра масс
    if (center1 > center2)
    {
        if (in_range<double>(center1, (2 * region.width()) / 3, region.width()) &&
                in_range<double>(center2, region.width() / 3, (2 * region.width()) / 3))
        {
            // Отсекаем тень слева
            region.set_left(region.get_left() + region.width() / 2);
            return true;
        }
    }
    else
    {
        if (in_range<double>(center1, 0, region.width() / 3) &&
                in_range<double>(center2, region.width() / 3, (2 * region.width()) / 3))
        {
            // Отсекаем тень справа
            region.set_right(region.get_left() + region.width() / 2);
            return true;
        }
    }
    return false;
}
////////////////////////////////////////////////////////////////////////////

template<class T>
const CZone* CFeinTrack::get_zone(const T& rect) const
{
    for (size_t i = 0; i < correct_zones.size(); ++i)
    {
        if (correct_zones[i].use_detection &&
                segments_superposition(correct_zones[i].left, correct_zones[i].right, rect.get_left(), rect.get_right()) &&
                segments_superposition(correct_zones[i].top, correct_zones[i].bottom, rect.get_top(), rect.get_bottom()))
        {
            return &correct_zones[i];
        }
    }
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
