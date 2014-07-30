#include <set>

#include "FeintrackManager.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	
	CFTCont::CFTCont()
	{
	}
	////////////////////////////////////////////////////////////////////////////
	
	CFTCont::~CFTCont()
	{
	}
	////////////////////////////////////////////////////////////////////////////
	
	CFTCont* CFTCont::create()
	{
		return new CFTCont();
	}
	////////////////////////////////////////////////////////////////////////////
	
	void CFTCont::destroy(CFTCont* &ft_cont)
	{
		if (ft_cont)
		{
			delete ft_cont;
            ft_cont = nullptr;
		}
	}
	////////////////////////////////////////////////////////////////////////////

	void CFTCont::set_use_feintrack(bool new_val)
	{
		fein_track.set_show_objects(new_val);
	}
	////////////////////////////////////////////////////////////////////////////

#if !ADV_OUT
    int CFTCont::frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type)
#else
    int CFTCont::frame_analyze(const uchar* buf, uint32_t width, uint32_t height, color_type buf_type, uchar* adv_buf_rgb24)
#endif
	{
#if !ADV_OUT
        int ret_val = fein_track.new_frame(buf, get_pixel_size<uint32_t>(buf_type) * width, width, height, buf_type);
#else
        int ret_val = fein_track.new_frame(buf, get_pixel_size<uint32_t>(buf_type) * width, width, height, buf_type, adv_buf_rgb24);
#endif
		return ret_val;
	}
	////////////////////////////////////////////////////////////////////////////

	void CFTCont::set_config(const void* config_struct)
	{
		CFeintrackParams *ftp = (CFeintrackParams *)config_struct;
		if (!ftp)
			return;

		// Конфигурация finetrack'a
		apply_ft_profile(*ftp);

		// Конфигурация finedome
		ftp->get_channel_name(channel_name);
	}
	////////////////////////////////////////////////////////////////////////////
	
	void CFTCont::update_config(int channel_fps, const char* channel_name, void* config_struct)
	{
		CFeintrackParams *ftp = (CFeintrackParams *)config_struct;
		if (!ftp)
			return;

		ftp->set_fps(channel_fps);
		ftp->set_channel_name(channel_name);
	}
	////////////////////////////////////////////////////////////////////////////
	
	void CFTCont::get_config(void* config_struct)
	{
		CFeintrackParams *ftp = (CFeintrackParams *)config_struct;
		if (!ftp)
			return;

		// Конфигурация finetrack'a
		fill_ft_profile(*ftp);

		// Конфигурация finedome
		ftp->set_channel_name(channel_name.c_str());
	}
	////////////////////////////////////////////////////////////////////////////

	void CFTCont::apply_ft_profile(const CFeintrackParams &params)
	{
		fein_track.set_sensitivity(params.get_sensitivity());
		fein_track.set_fps(params.get_fps());
		fein_track.set_show_objects(params.get_show_objects());
		fein_track.set_use_cuda(params.get_use_cuda(), params.get_cuda_device());
		fein_track.set_bgrnd_type(params.get_bgrnd_type());
		fein_track.set_use_recognition(params.get_use_recognition());
		fein_track.set_use_morphology(params.get_use_morphology());
		fein_track.set_show_left_objects(params.get_show_left_objects());
		fein_track.set_show_trajectory(params.get_show_trajectory());
		fein_track.set_selection_time(params.get_selection_time());
		fein_track.set_min_region_width(params.get_min_region_width());
		fein_track.set_min_region_height(params.get_min_region_height());
		fein_track.set_analyze_area(params.get_analyze_area());
		fein_track.set_use_square_segmentation(params.get_use_square_segmentation());
		fein_track.set_detect_patches_of_sunlight(params.get_detect_patches_of_sunlight());
		fein_track.set_cut_shadows(params.get_cut_shadows());

		fein_track.set_left_object_time1_sec(params.get_left_object_time1_sec());
		fein_track.set_left_object_time2_sec(params.get_left_object_time2_sec());
		fein_track.set_left_object_time3_sec(params.get_left_object_time3_sec());

		zones_cont zones;
		params.get_zones(zones);
		fein_track.set_zones_list(zones);
		lines_cont lines;
		params.get_lines(lines);
		fein_track.set_lines_list(lines);
	}
	////////////////////////////////////////////////////////////////////////////
	
	void CFTCont::fill_ft_profile(CFeintrackParams &params) const
	{
		params.set_sensitivity(fein_track.get_sensitivity());
		params.set_show_objects(fein_track.get_show_objects());
		params.set_use_cuda(fein_track.get_use_cuda(), fein_track.get_cuda_device());
		params.set_bgrnd_type(fein_track.get_bgrnd_type());
		params.set_use_recognition(fein_track.get_use_recognition());
		params.set_use_morphology(fein_track.get_use_morphology());
		params.set_show_left_objects(fein_track.get_show_left_objects());
		params.set_show_trajectory(fein_track.get_show_trajectory());
		params.set_selection_time(fein_track.get_selection_time());
		params.set_min_region_width(fein_track.get_min_region_width());
		params.set_min_region_height(fein_track.get_min_region_height());
		params.set_analyze_area(fein_track.get_analyze_area());
		params.set_use_square_segmentation(fein_track.get_use_square_segmentation());
		params.set_detect_patches_of_sunlight(fein_track.get_detect_patches_of_sunlight());
		params.set_cut_shadows(fein_track.get_cut_shadows());

		params.set_left_object_time1_sec(fein_track.get_left_object_time1_sec());
		params.set_left_object_time2_sec(fein_track.get_left_object_time2_sec());
		params.set_left_object_time3_sec(fein_track.get_left_object_time3_sec());

		zones_cont zones;
		fein_track.get_zones_list(zones);
		params.set_zones(zones);

		lines_cont lines;
		fein_track.get_lines_list(lines);
		params.set_lines(lines);
	}
	////////////////////////////////////////////////////////////////////////////
	
	bool CFTCont::activate_profile(const char* profile_name)
	{
		ft_params.set_curr_profile(profile_name);
		apply_ft_profile(ft_params);
		return true;
	}
	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////
	
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
