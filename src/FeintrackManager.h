#pragma once

//#define DBG_OUT           // ����� ���������� ��������� ��� AutoFeinDome

#include "feintrack_params.h"
#include "feintrack.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	// ������ ���������� � feintrack'e � ���������� � ��� ���������� �������
	class CFTCont
	{
	private:
		CFTCont();
		~CFTCont();

		void apply_ft_profile(const CFeintrackParams &params); // ���������� ������� finetrack'a
		void fill_ft_profile(CFeintrackParams &params) const;  // ���������� ������� �������� ����������� finetrack'a

		cstring channel_name;                       // ��� ������, � �������� �������� feintrack

		CFeintrackParams ft_params;                 // ��������� Finetrack � FineDome

	public:
		CFeinTrack fein_track;                      // feintrack

		static CFTCont* create();                   // �������� �
		static void destroy(CFTCont* &ft_cont);     // �������� ������� ������

#if !ADV_OUT
        int frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type);                // ������ ���������� �����
#else
        int frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24); // ������ ���������� �����
#endif

		void set_config(const void* config_struct); // ������� ������������ Feintrack'a
		void update_config(int channel_fps, const char* channel_name, void* config_struct); // ��������� ������������ Feintrack'a
		void get_config(char* &config_str);         // ��������� ������������ Feintrack'a
		void get_config(void* config_struct);       // ��������� ������������ Feintrack'a

		void set_use_feintrack(bool new_val);       // ���������/���������� feintrack'a

		bool activate_profile(const char* profile_name); // ��������� ������� �������� finetrack'a
	};
	////////////////////////////////////////////////////////////////////////////
	
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
