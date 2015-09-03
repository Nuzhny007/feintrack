#pragma once

//#define DBG_OUT           // Вывод отладочной информаци для AutoFeinDome

#include "feintrack_params.h"
#include "feintrack.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////
namespace feintrack
{
	////////////////////////////////////////////////////////////////////////////
	// Хранит информацию о feintrack'e и сопряжённых с ним поворотных камерах
	class CFTCont
	{
    public:
        CFTCont();
        ~CFTCont();

		CFeinTrack fein_track;                      // feintrack

#if !ADV_OUT
        int frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type);                // Анализ очередного кадра
#else
        int frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24); // Анализ очередного кадра
#endif

        void set_config(const CFeintrackParams& config_struct); // Задание конфигурации Feintrack'a
        void get_config(CFeintrackParams& config_struct) const; // Получение конфигурации Feintrack'a

		void set_use_feintrack(bool new_val);       // Включение/выключение feintrack'a

		bool activate_profile(const char* profile_name); // Активация профиля настроек feintrack'a

    private:
        void apply_ft_profile(const CFeintrackParams &params); // Применение профиля feintrack'a
        void fill_ft_profile(CFeintrackParams &params) const;  // Заполнение профиля текущими установками feintrack'a

        CFeintrackParams ft_params;                 // Параметры Feintrack и FineDome
	};
	////////////////////////////////////////////////////////////////////////////
	
} //end namespace feintrack
////////////////////////////////////////////////////////////////////////////
