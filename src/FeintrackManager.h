#pragma once

//#define DBG_OUT           // Вывод отладочной информаци для AutoFeinDome

#include "feintrack_params.h"
#include "feintrack.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	// Хранит информацию о feintrack'e и сопряжённых с ним поворотных камерах
	class CFTCont
	{
	private:
		CFTCont();
		~CFTCont();

		void apply_ft_profile(const CFeintrackParams &params); // Применение профиля finetrack'a
		void fill_ft_profile(CFeintrackParams &params) const;  // Заполнение профиля текущими установками finetrack'a

		cstring channel_name;                       // Имя канала, к которому привязан feintrack

		CFeintrackParams ft_params;                 // Параметры Finetrack и FineDome

	public:
		CFeinTrack fein_track;                      // feintrack

		static CFTCont* create();                   // Создание и
		static void destroy(CFTCont* &ft_cont);     // удаление объекта класса

#if !ADV_OUT
        int frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type);                // Анализ очередного кадра
#else
        int frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24); // Анализ очередного кадра
#endif

		void set_config(const void* config_struct); // Задание конфигурации Feintrack'a
		void update_config(int channel_fps, const char* channel_name, void* config_struct); // Изменение конфигурации Feintrack'a
		void get_config(char* &config_str);         // Получение конфигурации Feintrack'a
		void get_config(void* config_struct);       // Получение конфигурации Feintrack'a

		void set_use_feintrack(bool new_val);       // Включение/выключение feintrack'a

		bool activate_profile(const char* profile_name); // Активация профиля настроек finetrack'a
	};
	////////////////////////////////////////////////////////////////////////////
	
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
