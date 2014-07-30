#pragma once

#include "feintrack_params.h"
#include "utils.h"
#include "feintrack_objects.h"
#include "segmentation.h"
#include "some_types.h"
////////////////////////////////////////////////////////////////////////////

namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////

    typedef uint32_t ft_param_t;                 // ��� �������������� ������������� ������ ������� �����
	////////////////////////////////////////////////////////////////////////////

	// ������� ����� ��� ���������� ��������� ����
	class CBackSubstraction
	{
	public:
		CBackSubstraction();
		virtual ~CBackSubstraction();

        virtual bool init(uint32_t width, uint32_t height, color_type buf_type, bool& use_cuda_); // ���������� true, ���� ������������� ���� ���������, ����� - false

		// ��������� ����
#if !ADV_OUT
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask) = 0;
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l) = 0;
#endif
#else
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24) = 0;
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24) = 0;
#endif
#endif

		// ��������� ���������� � �������
        virtual void update_statistic_in_region(const uchar* buf, uint32_t pitch, const CObjectRegion& region) = 0;

		// ������ �������� ����������� �������� � ������� ������� ������� ��������� ��������
        virtual void reset_statistic_in_region(const uchar* buf, uint32_t pitch, const CObjectRegion& region) = 0;

		bool get_detect_patches_of_sunlight() const;                           // ������������ �� �������� ������
		void set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_); // ���������/���������� ��������� ������

		virtual void set_sensitivity(int sens_level);         // ����� ������� ���������������� ��� ��������� ���� (�� 1 �� 100)
		virtual int get_sensitivity() const;                  // ��������� ������ ���������������� ��� ��������� ���� (�� 1 �� 100)

		virtual void set_fps(int new_fps);                    // ������� fps

		virtual void enable_back_update(bool enable_val);     // ����������/���������� ���������� ����

		virtual void set_show_objects(bool show_objects);     // ����������/�� ���������� �������

		virtual void set_use_cuda(bool use_cuda_);            // ������� ������������� CUDA

	protected:
		// ��������� �������� ��������������� ������� ������������ �����������
        static const ft_param_t alpha1;               // ��� ����������� ��������
        static const ft_param_t alpha2;               // ��� ������������������� ����������

        static const ft_param_t min_sigma_val;        // ����������� �
        static const ft_param_t max_sigma_val;        // ������������ �������� ��� ������������������� ���������� (������������ ��� ��������� ����)

        uint32_t frame_width;                         // ������
        uint32_t frame_height;                        // � ������ ������ ����� � ��������
		color_type curr_color_type;                   // ������� ��� ��������� ������������ �������������� �����

        ft_param_t epsilon;                           // �����, �� �������� ������������ �������������� ������� ��������� ��� ������� ����� (���������� ������������)

        int pixel_size;                               // ������ ������ ������� � ������

		bool use_cuda;                                // ������������ CUDA ��� ��������� ���� � ����������

		bool init_filter;                             // ������� �� �������������� ����������

		int fps;                                      // ���������� ������ � �������, ����������� �� ������. ���������� ���������� ������������ ��� � �������. ����� ����� ���������� �� ������.

		bool need_background_update;                  // ��������� �� ������ ������� �����

        bool is_patch_of_sunlight(const ft_param_t* float_src, const size_t pixel_size); // �������� �� ������ ������� ������ �����

	private:
        static const ft_param_t min_sens;             // ����������� �
        static const ft_param_t max_sens;             // ������������ �������� ������ ��� ��������� ���� (epsilon)

		bool detect_patches_of_sunlight;              // ������������ �������� ������
        static const ft_param_t sunlight_threshold;   // ����� �������� ������� ��� ����������� �����
	};
	////////////////////////////////////////////////////////////////////////////

	// ��������� ����������� �������������
	struct CNormParams
	{
		CNormParams()
			: mu(0), sigma(0)
		{
		}
        ft_param_t mu;       // ���������� �������
        ft_param_t sigma;    // ������������������ ����������

		// ������������� �������� ����������� �������� � ������� ����������������� �����������
        void recalc_mu(ft_param_t new_val, ft_param_t alpha)
		{
            mu = ((100 - alpha) * mu + alpha * new_val) / 100;
		}
		// ������������� �������� ������������������� ���������� � ������� ����������������� �����������
        void recalc_sigma(ft_param_t new_val, ft_param_t alpha)
		{
            sigma = static_cast<ft_param_t>(sqrt(
                        ((100 - alpha) * sqr(sigma) +
                        alpha * sqr(new_val - mu)) / 100
                        ) + 0.5);
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// ��������� ����������� ������������� ��� ������� (�� ������ ��������� ����� ���� ��������� ��������)
	template<size_t NORM_COUNT>
	struct CNormParamsPixel
	{
		CNormParams p[NORM_COUNT];  // ��������� ����������� ������������� ��� ������� ����� �������
        static const size_t PIXEL_VALUES = NORM_COUNT; ///< Count a significant values on pixel

		// ������������� �������� ����������� �������� ��� ���� ���������
        void recalc_mu(ft_param_t* new_val, ft_param_t alpha)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].recalc_mu(new_val[i], alpha);
			}
		}

		// ������������� �������� ������������������� ���������� ��� ���� ���������
        void recalc_sigma(ft_param_t* new_val, ft_param_t alpha, ft_param_t min_sigma_val, ft_param_t max_sigma_val)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].recalc_sigma(new_val[i], alpha);
				set_range(p[i].sigma, min_sigma_val, max_sigma_val);
			}
		}

		// �������� �������������� ������ ������� ����� �� ��������� �����
        void create_statistic(ft_param_t* new_val, ft_param_t curr_frame)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].mu += new_val[i];
				p[i].sigma += sqr(new_val[i] - p[i].mu / curr_frame);
			}
		}
		// ���������� �������� ������ ������� �����
        void end_create_statistic(ft_param_t* new_val, ft_param_t curr_frame, ft_param_t min_sigma_val, ft_param_t max_sigma_val)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].mu = (p[i].mu + new_val[i]) / curr_frame;

				p[i].sigma += sqr(new_val[i] - p[i].mu / curr_frame);
				p[i].sigma = sqrt(p[i].sigma / (curr_frame - 1));
				set_range(p[i].sigma, min_sigma_val, max_sigma_val);
			}
		}

		// �������� �� �������������� ������� ������� �����
        bool is_back(ft_param_t *new_val, ft_param_t *eps) const
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
                if (eps[i] * p[i].sigma < abs(p[i].mu - new_val[i]))
                {
					return false;
                }
			}
			return true;
		}

		// ������� �������� ������
        void set_mu_sigma(ft_param_t* new_val, ft_param_t new_sigma)
		{
			for (size_t i = 0; i < NORM_COUNT; ++i)
			{
				p[i].mu = new_val[i];
				p[i].sigma = new_sigma;
			}
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// ��������� ���� �� ������ ������������� ������� ������� ��������� ��������� ������������� ��������
	class CNormBackSubstraction: public CBackSubstraction
	{
	public:
		CNormBackSubstraction();
		~CNormBackSubstraction();

        bool init(uint32_t width, uint32_t height, color_type buf_type, bool& use_cuda_); // ���������� true, ���� ������������� ���� ���������, ����� - false

		// ��������� ����
#if !ADV_OUT
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l);
#endif
#else
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24);
#endif
#endif

        void update_statistic_in_region(const uchar* buf, uint32_t pitch, const CObjectRegion& region); // ��������� ���������� � �������
        void reset_statistic_in_region(const uchar* buf, uint32_t pitch, const CObjectRegion& region);  // ������ �������� ����������� �������� � ������� ������� ������� ��������� ��������

		void set_fps(int new_fps);                    // ������� fps
		void set_show_objects(bool show_objects);     // ����������/�� ���������� �������

	private:
		typedef CNormParamsPixel<3> rgb_param_type;            //
		typedef CNormParamsPixel<1> gray_param_type;           //
		typedef std::vector<rgb_param_type> rgb_params_cont;   // 
		typedef std::vector<gray_param_type> gray_params_cont; // 

		int contrast_time;                            // ����� ��� �������� ��������� �����
		int contrast_frame;                           // ������� ������ ��� ������� ������� �� contrast_time
		static const float_t contrast_threshold;      // ����� �������� ���������, ���� �������� ����������� �������� �����

		bool use_shadow_detector;                     // ������������ �������� �����

		rgb_params_cont rgb_params;                   // ������ �������� frame_width * frame_height, � ������� �������� �������������� ������ � ������ �������
		gray_params_cont gray_params;                 // ������ �������� frame_width * frame_height, � ������� �������� �������������� ������ � ������ �������

#ifdef USE_CUDA
        CCudaBuf<BGRXf, false> h_frame_bgrxf;         // ����� ����� ��� ����������� � �����������
        CCudaBuf<int32_t, true> d_bgr32;              // ����������� ��� ����
		CCudaBuf<float, true> d_params_b_mu;          // ����������� ��� ��������� ������ ������� �����
		CCudaBuf<float, true> d_params_b_sigma;       // ����������� ��� ��������� ������ ������� �����
		CCudaBuf<float, true> d_params_g_mu;          // ����������� ��� ��������� ������ ������� �����
		CCudaBuf<float, true> d_params_g_sigma;       // ����������� ��� ��������� ������ ������� �����
		CCudaBuf<float, true> d_params_r_mu;          // ����������� ��� ��������� ������ ������� �����
		CCudaBuf<float, true> d_params_r_sigma;       // ����������� ��� ��������� ������ ������� �����
#endif

		// ��������� ����
		template<class PARAMS_CONT>
#if !ADV_OUT
#ifdef USE_CUDA
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, PARAMS_CONT& params);
#endif
#else
#ifdef USE_CUDA
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#endif
#endif

		// ��������� ���������� � �������
		template<class PARAMS_CONT>
        void update_statistic_in_region(const uchar* buf, uint32_t pitch, PARAMS_CONT& params, const CObjectRegion& region);
		
		// ������ �������� ����������� �������� � ������� ������� ������� ��������� ��������
		template<class PARAMS_CONT>
        void reset_statistic_in_region(const uchar* buf, uint32_t pitch, PARAMS_CONT& params, const CObjectRegion& region);
	};
	////////////////////////////////////////////////////////////////////////////

	// ��������� ����������� ������������� + ��� ���������� ��� ����� ��������
	template<size_t NORM_COUNT>
	struct CNormWeightProcess: public CNormParamsPixel<NORM_COUNT>
	{
		CNormWeightProcess()
			: weight(0)
		{
		}

        float_t weight;       // ��� ��������
	};
	////////////////////////////////////////////////////////////////////////////

	// ������ ���������, ��������������� ������� �������
	template<size_t NORM_COUNT>
	struct CNormWeightParams3
	{
		CNormWeightParams3()
			: curr_proc(0), created_processes(1)
		{
		}

		static const size_t PROC_PER_PIXEL = 3;                   // ����� ��������� �� ������ �������
		CNormWeightProcess<NORM_COUNT> proc_list[PROC_PER_PIXEL]; // ��������� ������������� �� ������ �������
		size_t curr_proc;                                         // ������� �������
		size_t created_processes;                                 // ���������� ��� ��������� ���������

        static const size_t PIXEL_VALUES = NORM_COUNT; ///< Count a significant values on pixel

		// �������� �������������� ������ ������� ����� �� ��������� �����
        void create_statistic(ft_param_t* new_val, ft_param_t curr_frame)
		{
			for (size_t proc_ind = 0; proc_ind < PROC_PER_PIXEL; ++proc_ind)
			{
				proc_list[proc_ind].create_statistic(new_val, curr_frame);
			}
		}
		// ���������� �������� ������ ������� �����
        void end_create_statistic(ft_param_t* new_val, ft_param_t curr_frame, ft_param_t min_sigma_val, ft_param_t max_sigma_val)
		{
			for (size_t proc_ind = 0; proc_ind < PROC_PER_PIXEL; ++proc_ind)
			{
				proc_list[proc_ind].end_create_statistic(new_val, curr_frame, min_sigma_val, max_sigma_val);
			}
			proc_list[curr_proc].weight = 1;
		}

		// �������� �� �������������� ������� ������� �����
        bool is_back(ft_param_t *new_val, ft_param_t *eps, ft_param_t alpha1, ft_param_t alpha2, float_t alpha3, ft_param_t min_sigma_val, ft_param_t max_sigma_val, float_t weight_threshold)
		{
			bool find_process = false;

			for (size_t proc_ind = 0; proc_ind < created_processes; ++proc_ind)
			{
				// ���� �������, ������� ����� ������������� �������� �������� �������
				if (proc_list[proc_ind].is_back(new_val, eps))
				{
					// ������� ������ - �������� ��� ���������
					curr_proc = proc_ind;
					
					// ������ ���. �������� � ��������� ����������� � ������� ��������������� ������� ������������ �����������
					proc_list[curr_proc].recalc_mu(new_val, alpha1);
					proc_list[curr_proc].recalc_sigma(new_val, alpha2, min_sigma_val, max_sigma_val);

					find_process = true;
					break;
				}
			}
			if (!find_process) // ������� �� ������
			{
				// ������ ����� ������� ���,
				if (created_processes < PROC_PER_PIXEL)
				{
					++created_processes;
					curr_proc = created_processes - 1;

					proc_list[curr_proc].set_mu_sigma(new_val, min_sigma_val);

					find_process = true;
				}
				// ���� ���������� ��������� ����� PROC_PER_PIXEL, ���� ������� � ���������� �����
				else
				{
                    float_t min_weight = proc_list[0].weight;
					size_t min_proc = 0;
					for (size_t proc_ind = 1; proc_ind < created_processes; ++proc_ind)
					{
						if (proc_list[proc_ind].weight < min_weight)
						{
							min_proc = proc_ind;
							min_weight = proc_list[proc_ind].weight;
						}
					}
					curr_proc = min_proc;
					proc_list[curr_proc].set_mu_sigma(new_val, min_sigma_val);
				}
			}

			// ���������� ����� ���������
			if (find_process)
			{
				for (size_t proc_ind = 0; proc_ind < created_processes; ++proc_ind)
				{
					proc_list[proc_ind].weight = (1 - alpha3) * proc_list[proc_ind].weight + alpha3 * ((proc_ind == curr_proc) ? 1 : 0);
				}
			}

			return proc_list[curr_proc].weight > weight_threshold;
		}

		// ������� �������� ������
        void set_mu_sigma(ft_param_t* new_val, ft_param_t new_sigma)
		{
			proc_list[curr_proc].set_mu_sigma(new_val, new_sigma);
		}
	};
	////////////////////////////////////////////////////////////////////////////

	// ��������� ���� �� ������ ������������� ������� ������� ��������� ��������� ������������� ��������
	class CGaussianMixtureBackSubstr: public CBackSubstraction
	{
	public:
		CGaussianMixtureBackSubstr();
		~CGaussianMixtureBackSubstr();

        bool init(uint32_t width, uint32_t height, color_type buf_type, bool& use_cuda_); // ���������� true, ���� ������������� ���� ���������, ����� - false

		// ��������� ����
#if !ADV_OUT
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l);
#endif
#else
#ifdef USE_CUDA
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24);
#else
        virtual int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24);
#endif
#endif

        void update_statistic_in_region(const uchar* buf, uint32_t pitch, const CObjectRegion& region); // ��������� ���������� � �������
        void reset_statistic_in_region(const uchar* buf, uint32_t pitch, const CObjectRegion& region);  // ������ �������� ����������� �������� � ������� ������� ������� ��������� ��������

		void set_show_objects(bool show_objects);              // ����������/�� ���������� �������

	private:
		typedef CNormWeightParams3<3> rgb_param_type;          //
		typedef CNormWeightParams3<1> gray_param_type;         //
		typedef std::vector<rgb_param_type> rgb_params_cont;   // 
		typedef std::vector<gray_param_type> gray_params_cont; // 

		rgb_params_cont rgb_params;                            // ������ �������� frame_width * frame_height, � ������� �������� �������������� ������ � ������ �������
		gray_params_cont gray_params;                          // ������ �������� frame_width * frame_height, � ������� �������� �������������� ������ � ������ �������

#ifdef USE_CUDA
        CCudaBuf<BGRXf, false> h_frame_bgrxf;                  // ����� ����� ��� ����������� � �����������
        CCudaBuf<int32_t, true> d_bgr32;                       // ����������� ��� ����
        CCudaBuf<int32_t, true> d_curr_processes;              // ����������� � ��������� ������� ��������� ��� ������� �������
        CCudaBuf<int32_t, true> d_created_processes;           // ����������� � ������ ��������� ��������� ��� ������� �������
		CCudaBuf<BgrndProcess, true> d_process1;               // ����������� � ����������� ������ ���� ��� ������� ��������
		CCudaBuf<BgrndProcess, true> d_process2;               // ����������� � ����������� ������ ���� ��� ������� ��������
		CCudaBuf<BgrndProcess, true> d_process3;               // ����������� � ����������� ������ ���� ��� �������� ��������
#endif

        static const float_t alpha3;                           // �������� �������� ��������������� ������� ������������ ����������� ��� ���� ��������
        static const float_t weight_threshold;                 // ����� ��� ���� ��������

		// ��������� ����
		template<class PARAMS_CONT>
#if !ADV_OUT
#ifdef USE_CUDA
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, PARAMS_CONT& params);
#endif
#else
#ifdef USE_CUDA
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#else
        int background_substraction(int& curr_frame, const uchar* buf, uint32_t pitch, mask_cont& pixels_l, PARAMS_CONT& params, uchar* adv_buf_rgb24);
#endif
#endif

		// ������ �������� ����������� �������� � ������� ������� ������� ��������� ��������
		template<class PARAMS_CONT>
        void reset_statistic_in_region(const uchar* buf, uint32_t pitch, PARAMS_CONT& params, const CObjectRegion& region);
	};
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
