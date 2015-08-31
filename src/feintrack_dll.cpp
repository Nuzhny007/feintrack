#include "feintrack_dll.h"
#include "FeintrackManager.h"
#include "utils.h"

namespace vl_feintrack
{
////////////////////////////////////////////////////////////////////////////

void* AddFeintrack()
{
    CFTCont* ft_cont = CFTCont::create();
    return reinterpret_cast<void*>(ft_cont);
}
////////////////////////////////////////////////////////////////////////////

void DelFeintrack(void* feintrack)
{
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    CFTCont::destroy(ft_cont);
}
////////////////////////////////////////////////////////////////////////////

#if !ADV_OUT
int FeintrackFrameAnalyze(void* feintrack, const uchar* buf, int width, int height, color_type buf_type)
#else
int FeintrackFrameAnalyze(void* feintrack, const uchar* buf, int width, int height, color_type buf_type, uchar* adv_buf_rgb24)
#endif
{
    if (!feintrack)
        return 0;

    int ret_val(0);

#if 0
    static vl_feintrack::CPerfTimer pt;
    pt.start_timer();
#endif

    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);

#if !ADV_OUT
    ret_val = ft_cont->frame_analyze(buf, width, height, buf_type);
#else
    ret_val = ft_cont->frame_analyze(buf, width, height, buf_type, adv_buf_rgb24);
#endif

    return ret_val;
}
////////////////////////////////////////////////////////////////////////////

// Получение списка объектов, обнаруженных на пoследнем кадре
void GetObjects(void* feintrack, vl_feintrack::CObjRect* &rect_arr, size_t& rect_count)
{
    if (!feintrack)
    {
        rect_arr = NULL;
        rect_count = 0;
        return;
    }
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->fein_track.get_objects(rect_arr, rect_count);
}
////////////////////////////////////////////////////////////////////////////

// Получение списка идентификаторов удалённых на последнем кадре объектов
void GetDelObjects(void* feintrack, unsigned int* &uids_arr, size_t& uids_count)
{
    if (!feintrack)
    {
        uids_arr = NULL;
        uids_count = 0;
        return;
    }
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->fein_track.get_del_objects(uids_arr, uids_count);
}
////////////////////////////////////////////////////////////////////////////

// Получение списка оставленных предметов
void GetLeftObjects(void* feintrack, vl_feintrack::CLeftObjRect* &rect_arr, size_t& rect_count)
{
    if (!feintrack)
    {
        rect_arr = NULL;
        rect_count = 0;
        return;
    }
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->fein_track.get_left_objects(rect_arr, rect_count);
}
////////////////////////////////////////////////////////////////////////////

// Получение конфигурации Feintrack'а
bool GetFeintrackConfigStruct(void* feintrack, void* config_struct)
{
    if (!feintrack)
        return false;

    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->get_config(config_struct);
    return true;
}
////////////////////////////////////////////////////////////////////////////

// Задание конфигурации Feintrack'а
void SetFeintrackConfigStruct(void* feintrack, const void* config_struct)
{
    if (!feintrack)
        return;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->set_config(config_struct);
}
////////////////////////////////////////////////////////////////////////////

// Изменение конфигурации Feintrack'а: задание нового uid и fps (т.к. они могут измениться при перезапуске Видеолокатора)
void UpdateFeintrackConfigStruct(void* feintrack, int channel_fps, const char* channel_name, void* config_struct)
{
    if (feintrack)
    {
        CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
        ft_cont->update_config(channel_fps, channel_name, config_struct);
    }
}
////////////////////////////////////////////////////////////////////////////

// Используется ли детектирование объектов на Feintrack'e
bool GetUseFeintrack(void* feintrack)
{
    if (!feintrack)
        return false;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);

    return ft_cont->fein_track.get_show_objects();
}
////////////////////////////////////////////////////////////////////////////

// Использовать детектирование объектов в Feintrack'e
void SetUseFeintrack(void* feintrack, bool new_val)
{
    if (!feintrack)
        return;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->set_use_feintrack(new_val);
}
////////////////////////////////////////////////////////////////////////////

// Разрешение/запрещение обновления фона в feintrack'e
void EnableBackUpdate(void* feintrack, bool enable_val)
{
    if (!feintrack)
        return;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->fein_track.enable_back_update(enable_val);
}
////////////////////////////////////////////////////////////////////////////

// Активация профиля настроек finetrack'a
bool ActivateProfile(void* feintrack, const char* profile_name)
{
    if (!feintrack)
        return false;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    return ft_cont->activate_profile(profile_name);
}
////////////////////////////////////////////////////////////////////////////

// Задание координат единственного объекта, который будет отображаться на кадре
void SetOneObject(void* feintrack, unsigned int obj_uid, int left, int right, int top, int bottom)
{
    if (!feintrack)
        return;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->fein_track.set_one_object(obj_uid, left, right, top, bottom);
}
////////////////////////////////////////////////////////////////////////////

// Получение набора точек, принадлежащих объекту
void GetObjectPoints(void* feintrack, size_t obj_ind, POINTF* points, size_t& max_points)
{
    if (!feintrack)
        return;
    CFTCont* ft_cont = reinterpret_cast<CFTCont*>(feintrack);
    ft_cont->fein_track.get_object_points(obj_ind, points, max_points);
}
////////////////////////////////////////////////////////////////////////////
} // end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
