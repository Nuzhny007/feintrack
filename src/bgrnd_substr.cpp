#include "bgrnd_substr.h"

////////////////////////////////////////////////////////////////////////////
namespace vl_feintrack
{
////////////////////////////////////////////////////////////////////////////

const ft_float_t CBackSubstraction::min_sens = (ft_float_t)1.;   // ����������� �
const ft_float_t CBackSubstraction::max_sens = (ft_float_t)10.0; // ������������ �������� ������ ��� ��������� ���� (epsilon)

// ��������� �������� ��������������� ������� ������������ �����������
const ft_float_t CBackSubstraction::alpha1 = (ft_float_t)0.2; // ��� ����������� ��������
const ft_float_t CBackSubstraction::alpha2 = (ft_float_t)0.1; // ��� ������������������� ����������

const ft_float_t CBackSubstraction::max_sigma_val = (ft_float_t)30.0; // ����������� �
const ft_float_t CBackSubstraction::min_sigma_val = (ft_float_t)3.0;  // ������������ �������� ��� ������������������� ���������� (������������ ��� ��������� ����)

const ft_float_t CBackSubstraction::sunlight_threshold = (ft_float_t)150.; // ����� �������� ������� ��� ����������� �����
////////////////////////////////////////////////////////////////////////////

CBackSubstraction::CBackSubstraction()
{
    frame_width = 0;
    frame_height = 0;
    curr_color_type = buf_rgb24;

    detect_patches_of_sunlight = false;

    epsilon = (1.5f * (max_sens + min_sens)) / 4.0f;
    use_cuda = false;

    PIXEL_SIZE = 3;
    fps = 25;
    need_background_update = true;
}
////////////////////////////////////////////////////////////////////////////

CBackSubstraction::~CBackSubstraction()
{
}
////////////////////////////////////////////////////////////////////////////

bool CBackSubstraction::get_detect_patches_of_sunlight() const
{
    return detect_patches_of_sunlight;
}
////////////////////////////////////////////////////////////////////////////

void CBackSubstraction::set_detect_patches_of_sunlight(bool detect_patches_of_sunlight_)
{
    detect_patches_of_sunlight = detect_patches_of_sunlight_;
}
////////////////////////////////////////////////////////////////////////////

bool CBackSubstraction::is_patch_of_sunlight(const ft_float_t* float_src)
{
    if (detect_patches_of_sunlight)
    {
        if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
            return (float_src[0] > sunlight_threshold) && (float_src[1] > sunlight_threshold) && (float_src[2] > sunlight_threshold);
        else
            return (float_src[0] > sunlight_threshold);
    }
    else
        return false;
}
////////////////////////////////////////////////////////////////////////////

void CBackSubstraction::set_sensitivity(int sens_level)
{
    set_range(sens_level, 1, 100);
    epsilon = min_sens + ((100 - sens_level) * (max_sens - min_sens)) / 100.0f;
}
////////////////////////////////////////////////////////////////////////////

int CBackSubstraction::get_sensitivity() const
{
    return 100 - (int)((100.0 * (epsilon - min_sens)) / (max_sens - min_sens));
}
////////////////////////////////////////////////////////////////////////////

void CBackSubstraction::set_show_objects(bool show_objects)
{
    // ���� ����������� �������� �� �����, �� ��������� ��������� ������ � ������� ����������
    if (!show_objects)
    {
        frame_width = 0;
        frame_height = 0;
    }
}
////////////////////////////////////////////////////////////////////////////

void CBackSubstraction::set_use_cuda(bool use_cuda_)
{
    use_cuda = use_cuda_;
}
////////////////////////////////////////////////////////////////////////////

void CBackSubstraction::set_fps(int new_fps)
{
    fps = new_fps;
}
////////////////////////////////////////////////////////////////////////////

void CBackSubstraction::enable_back_update(bool enable_val)
{
    need_background_update = enable_val;
}
////////////////////////////////////////////////////////////////////////////

bool CBackSubstraction::init(uint width, uint height, color_type buf_type, bool& /*use_cuda_*/)
{
    PIXEL_SIZE = get_pixel_size<int>(buf_type);

    if ((frame_width != width) || (frame_height != height) || (curr_color_type != buf_type))
    {
        frame_width = width;
        frame_height = height;
        curr_color_type = buf_type;

        return true;
    }
    return false;
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

const float_t CNormBackSubstraction::contrast_threshold = 0.45;

////////////////////////////////////////////////////////////////////////////

CNormBackSubstraction::CNormBackSubstraction()
    : CBackSubstraction()
{
    use_shadow_detector = true;
    init_filter = false;
}
////////////////////////////////////////////////////////////////////////////

CNormBackSubstraction::~CNormBackSubstraction()
{
}
////////////////////////////////////////////////////////////////////////////

void CNormBackSubstraction::set_show_objects(bool show_objects)
{
    CBackSubstraction::set_show_objects(show_objects);

    // ���� ����������� �������� �� �����, �� ��������� ��������� ������ � ������� ����������
    if (!show_objects)
    {
        rgb_params.clear();
        gray_params.clear();

        init_filter = false;
    }
}
////////////////////////////////////////////////////////////////////////////

void CNormBackSubstraction::set_fps(int new_fps)
{
    CBackSubstraction::set_fps(new_fps);

    contrast_time = 10 * 60 * fps;
    contrast_frame = contrast_time - 1;
}
////////////////////////////////////////////////////////////////////////////

bool CNormBackSubstraction::init(uint width, uint height, color_type buf_type, bool& use_cuda_)
{
    if (CBackSubstraction::init(width, height, buf_type, use_cuda_) ||
            (((buf_type == buf_rgb24) || (buf_type == buf_rgb32)) ? rgb_params.empty() : gray_params.empty()))
    {
        const size_t pixels_count = frame_width * frame_height;

        rgb_params.clear();
        gray_params.clear();
        if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
            rgb_params.resize(pixels_count);
        else
            gray_params.resize(pixels_count);

        if (use_cuda)
        {
#ifdef USE_CUDA
            bool success_malloc = true;
            success_malloc &= h_frame_bgrxf.malloc(pixels_count);
            success_malloc &= d_bgr32.malloc(pixels_count);

            success_malloc &= d_params_b_mu.malloc(pixels_count);
            success_malloc &= d_params_b_sigma.malloc(pixels_count);
            success_malloc &= d_params_g_mu.malloc(pixels_count);
            success_malloc &= d_params_g_sigma.malloc(pixels_count);
            success_malloc &= d_params_r_mu.malloc(pixels_count);
            success_malloc &= d_params_r_sigma.malloc(pixels_count);

            if (!success_malloc)
            {
                use_cuda = false;
                use_cuda_ = use_cuda;
            }
#else
            assert(false);
#endif
        }

        init_filter = false;

        contrast_frame = contrast_time - 1;

        return true;
    }
    return false;
}
////////////////////////////////////////////////////////////////////////////

#ifndef ADV_OUT
#ifdef USE_CUDA
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask)
#else
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l)
#endif
#else
#ifdef USE_CUDA
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24)
#else
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24)
#endif
#endif
{
#ifndef ADV_OUT
#ifdef USE_CUDA
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, rgb_params);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, gray_params);
#else
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, rgb_params);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, gray_params);
#endif
#else
#ifdef USE_CUDA
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, rgb_params, adv_buf_rgb24);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, gray_params, adv_buf_rgb24);
#else
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, rgb_params, adv_buf_rgb24);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, gray_params, adv_buf_rgb24);
#endif
#endif
}
////////////////////////////////////////////////////////////////////////////

template<class PARAMS_CONT>
#ifndef ADV_OUT
#ifdef USE_CUDA
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params)
#else
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params)
#endif
#else
#ifdef USE_CUDA
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params, uchar* adv_buf_rgb24)
#else
int CNormBackSubstraction::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params, uchar* adv_buf_rgb24)
#endif
#endif
{
    // �������� � ������� 1-� �������: ��������� ���������� ������� � ������������������ ����������
    if (!init_filter && (curr_frame < fps))
    {
        ++curr_frame;

        auto* par = &params[0];
        const uchar* pbuf = buf;
        for (uint y = 0; y < frame_height; ++y)
        {
            for (uint x = 0; x < frame_width; ++x)
            {
                if (curr_frame == fps)
                    par->end_create_statistic(pbuf, (ft_float_t)curr_frame, min_sigma_val, max_sigma_val);
                else
                    par->create_statistic(pbuf, (ft_float_t)curr_frame);

                pbuf += PIXEL_SIZE;
                ++par;
            }
            pbuf += pitch - frame_width * PIXEL_SIZE;
        }

        if (curr_frame == fps)
        {
            if (use_cuda)
            {
#ifdef USE_CUDA
                // ������ ��� ������ �������������� ������
                switch (curr_color_type)
                {
                case buf_rgb24:
                case buf_rgb32:
                {
                    std::vector<float> h_params_b_mu(params.size());
                    std::vector<float> h_params_b_sigma(params.size());
                    std::vector<float> h_params_g_mu(params.size());
                    std::vector<float> h_params_g_sigma(params.size());
                    std::vector<float> h_params_r_mu(params.size());
                    std::vector<float> h_params_r_sigma(params.size());

                    for (size_t i = 0, stop = params.size(); i < stop; ++i)
                    {
                        h_params_b_mu[i] = params[i].p[0].mu;
                        h_params_b_sigma[i] = params[i].p[0].sigma;
                        h_params_g_mu[i] = params[i].p[1].mu;
                        h_params_g_sigma[i] = params[i].p[1].sigma;
                        h_params_r_mu[i] = params[i].p[2].mu;
                        h_params_r_sigma[i] = params[i].p[2].sigma;
                    }

                    cudaMemcpy(d_params_b_mu.buf, &h_params_b_mu[0], d_params_b_mu.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_params_b_sigma.buf, &h_params_b_sigma[0], d_params_b_sigma.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_params_g_mu.buf, &h_params_g_mu[0], d_params_g_mu.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_params_g_sigma.buf, &h_params_g_sigma[0], d_params_g_sigma.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_params_r_mu.buf, &h_params_r_mu[0], d_params_r_mu.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_params_r_sigma.buf, &h_params_r_sigma[0], d_params_r_sigma.buf_size, cudaMemcpyHostToDevice);
                    break;
                }

                case buf_gray:
                {
                    std::vector<float> h_params_mu(params.size());
                    std::vector<float> h_params_sigma(params.size());

                    for (size_t i = 0, stop = params.size(); i < stop; ++i)
                    {
                        h_params_mu[i] = params[i].p[0].mu;
                        h_params_sigma[i] = params[i].p[0].sigma;
                    }

                    cudaMemcpy(d_params_b_mu.buf, &h_params_mu[0], d_params_b_mu.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_params_b_sigma.buf, &h_params_sigma[0], d_params_b_sigma.buf_size, cudaMemcpyHostToDevice);
                    break;
                }
                }
#else
                assert(false);
#endif
            }
            curr_frame = 0;
            init_filter = true;
        }
        return 0;
    }

    if (++contrast_frame == contrast_time)
    {
        // ���������� �� ������������ �������� �����
        contrast_frame = 0;
        use_shadow_detector = (get_contrast_rgb24(buf, pitch, frame_width, frame_height) < contrast_threshold);
    }

    auto* par = &params[0];
    const uchar* pbuf = buf;

    // ��� � ������� ��������� ���������� - ��������� ���������� ����
    bool curr_background_update = (curr_frame % fps == fps - 1) && need_background_update;

    // ������������ (0.587 / 0.144) � (0.299 / 0.144) ������ ��� ����������� �������� �����
    // �� �������� ����� �� ������� �������� RGB->YUV ��� ���������� �������
    ft_float_t tmp_eps[3];
    if (use_shadow_detector)
    {
        tmp_eps[0] = epsilon;                        // b
        tmp_eps[1] = (0.587f / 0.144f) * tmp_eps[0]; // g
        tmp_eps[2] = (0.299f / 0.144f) * tmp_eps[0]; // r
    }
    else
    {
        tmp_eps[0] = epsilon;    // b
        tmp_eps[1] = tmp_eps[0]; // g
        tmp_eps[2] = tmp_eps[0]; // r
    }

    if (use_cuda)
    {
#ifdef USE_CUDA
        // ����������� ����� � ����������� �� 4 ����� ������
        switch (curr_color_type)
        {
        case buf_rgb24:
            copy_24to32((uchar*)h_frame_bgrxf.buf, 4 * frame_width, pbuf, frame_width, frame_height);
            break;

        case buf_gray:
            copy_gray_to_float((float*)h_frame_bgrxf.buf, pbuf, frame_width, frame_height);
            break;

        case buf_rgb32:
            memcpy((uchar*)h_frame_bgrxf.buf, pbuf, 4 * frame_width * frame_height);
            break;
        }

        // ����������� ����� � �����������
        if (cudaMemcpy(d_bgr32.buf, h_frame_bgrxf.buf, d_bgr32.buf_size, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            set_show_objects(false);
            return 0;
        }

        // ��������� ��������� ����
        if (curr_background_update)
        {
            PixelUpdateParams pup = { alpha1, alpha2, min_sigma_val, max_sigma_val };
            if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
                back_substraction_upd(d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, d_params_g_mu.buf, d_params_g_sigma.buf, d_params_r_mu.buf, d_params_r_sigma.buf, d_mask.buf, frame_width, frame_height, tmp_eps[0], tmp_eps[1], tmp_eps[2], pup);
            else
                back_substraction_gray_upd((float *)d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, d_mask.buf, frame_width, frame_height, tmp_eps[0], pup);
        }
        else
        {
            if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
                back_substraction(d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, d_params_g_mu.buf, d_params_g_sigma.buf, d_params_r_mu.buf, d_params_r_sigma.buf, d_mask.buf, frame_width, frame_height, tmp_eps[0], tmp_eps[1], tmp_eps[2]);
            else
                back_substraction_gray((float *)d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, d_mask.buf, frame_width, frame_height, tmp_eps[0]);
        }
#else
        assert(false);
#endif
    }
    else
    {
#ifdef ADV_OUT
        uchar* adv_dest_buf = adv_buf_rgb24;
#endif

        ft_float_t float_src[3];

        // ������ ��������� ��������� ����
        size_t i = 0;
        for (uint y = 0; y < frame_height; ++y)
        {
            for (uint x = 0; x < frame_width; ++x, ++i)
            {
#ifdef ADV_OUT
                adv_dest_buf[0] = (uchar)par->p[0].mu;
                if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
                {
                    adv_dest_buf[1] = (uchar)par->p[1].mu;
                    adv_dest_buf[2] = (uchar)par->p[2].mu;
                }
                else
                {
                    adv_dest_buf[1] = (uchar)par->p[0].mu;
                    adv_dest_buf[2] = (uchar)par->p[0].mu;
                }
#endif

                // ������������� �������
                float_src[0] = (ft_float_t)pbuf[0];
                if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
                {
                    float_src[1] = (ft_float_t)pbuf[1];
                    float_src[2] = (ft_float_t)pbuf[2];
                }

                if (par->is_back(float_src, tmp_eps) || is_patch_of_sunlight(float_src))
                {
                    // ������� ����������� ������� �����
                    pixels_l[i] = 0;

                    if (curr_background_update)
                    {
                        // ���������� ���������� ������ ������� �����
                        par->recalc_mu(float_src, alpha1);
                        par->recalc_sigma(float_src, alpha2, min_sigma_val, max_sigma_val);
                    }
                }
                else
                {
                    // ������� ����������� ��������� �����
                    pixels_l[i] = 1;

#ifdef ADV_OUT // ������� ��� ������� ��������� ����� ������ ������
#if 1
                    adv_dest_buf[0] = 0;
                    adv_dest_buf[1] = 0;
                    adv_dest_buf[2] = 0xff;
#endif
#endif
                }
                pbuf += PIXEL_SIZE;
                ++par;
#ifdef ADV_OUT
                adv_dest_buf += 3;
#endif
            }
            pbuf += pitch - frame_width * PIXEL_SIZE;
        }
    }
    return 1;
}
////////////////////////////////////////////////////////////////////////////

void CNormBackSubstraction::update_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region)
{
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        update_statistic_in_region(buf, pitch, rgb_params, region);
    else
        update_statistic_in_region(buf, pitch, gray_params, region);
}
////////////////////////////////////////////////////////////////////////////

template<class PARAMS_CONT>
void CNormBackSubstraction::update_statistic_in_region(const uchar* buf, uint pitch, PARAMS_CONT& params, const CObjectRegion& region)
{
    if (use_cuda)
    {
#ifdef USE_CUDA
        PixelUpdateParams pup = { alpha1, alpha2, min_sigma_val, max_sigma_val };
        if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
            update_statistic(d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, d_params_g_mu.buf, d_params_g_sigma.buf, d_params_r_mu.buf, d_params_r_sigma.buf, frame_width, region.get_left(), region.get_right(), region.get_top(), region.get_bottom(), pup);
        else
            update_statistic_gray((float *)d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, frame_width, region.get_left(), region.get_right(), region.get_top(), region.get_bottom(), pup);
#else
        assert(false);
#endif
    }
    else
    {
        // ������������� ������������� ������� � � ������ ������� ��������� ����������
        int w1 = pitch - PIXEL_SIZE * region.width();
        int w2 = frame_width - region.width();

        buf += PIXEL_SIZE * region.get_left() + pitch * region.get_top();

        auto* par = &params[region.get_left() + frame_width * region.get_top()];

        for (int y = region.get_top(); y <= region.get_bottom(); ++y)
        {
            for (int x = region.get_left(); x <= region.get_right(); ++x)
            {
                par->recalc_mu(buf, 0.2f * alpha1);
                par->recalc_sigma(buf, 0.2f * alpha2, min_sigma_val, max_sigma_val);

                ++par;
                buf += PIXEL_SIZE;
            }
            par += w2;
            buf += w1;
        }
    }
}
////////////////////////////////////////////////////////////////////////////

void CNormBackSubstraction::reset_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region)
{
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        reset_statistic_in_region(buf, pitch, rgb_params, region);
    else
        reset_statistic_in_region(buf, pitch, gray_params, region);
}
////////////////////////////////////////////////////////////////////////////

template<class PARAMS_CONT>
void CNormBackSubstraction::reset_statistic_in_region(const uchar* buf, uint pitch, PARAMS_CONT& params, const CObjectRegion& region)
{
    if (use_cuda)
    {
#ifdef USE_CUDA
        if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
            reset_statistic(d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, d_params_g_mu.buf, d_params_g_sigma.buf, d_params_r_mu.buf, d_params_r_sigma.buf, frame_width, region.get_left(), region.get_right(), region.get_top(), region.get_bottom(), max_sigma_val);
        else
            reset_statistic_gray((float *)d_bgr32.buf, d_params_b_mu.buf, d_params_b_sigma.buf, frame_width, region.get_left(), region.get_right(), region.get_top(), region.get_bottom(), max_sigma_val);
#else
        assert(false);
#endif
    }
    else
    {
        // ������������� ������������� ������� � �������� ������� ������� ����������� ����������� ��������
        int w1 = pitch - PIXEL_SIZE * region.width();
        int w2 = frame_width - region.width();

        buf += PIXEL_SIZE * region.get_left() + pitch * region.get_top();

        auto* par = &params[region.get_left() + frame_width * region.get_top()];

        for (int y = region.get_top(); y <= region.get_bottom(); ++y)
        {
            for (int x = region.get_left(); x <= region.get_right(); ++x)
            {
                par->set_mu_sigma(buf, max_sigma_val);

                ++par;
                buf += PIXEL_SIZE;
            }
            par += w2;
            buf += w1;
        }
    }
}
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

const ft_float_t CGaussianMixtureBackSubstr::alpha3 = (ft_float_t)0.05;
const ft_float_t CGaussianMixtureBackSubstr::weight_threshold = (ft_float_t)0.2;
////////////////////////////////////////////////////////////////////////////

CGaussianMixtureBackSubstr::CGaussianMixtureBackSubstr()
    : CBackSubstraction()
{
    init_filter = false;
}
////////////////////////////////////////////////////////////////////////////

CGaussianMixtureBackSubstr::~CGaussianMixtureBackSubstr()
{
}
////////////////////////////////////////////////////////////////////////////

void CGaussianMixtureBackSubstr::set_show_objects(bool show_objects)
{
    CBackSubstraction::set_show_objects(show_objects);

    // ���� ����������� �������� �� �����, �� ��������� ��������� ������ � ������� ����������
    if (!show_objects)
    {
        rgb_params.clear();
        gray_params.clear();

        init_filter = false;
    }
}
////////////////////////////////////////////////////////////////////////////

bool CGaussianMixtureBackSubstr::init(uint width, uint height, color_type buf_type, bool& use_cuda_)
{
    if (CBackSubstraction::init(width, height, buf_type, use_cuda_) ||
            (((buf_type == buf_rgb24) || (buf_type == buf_rgb32)) ? rgb_params.empty() : gray_params.empty()))
    {
        const size_t pixels_count = frame_width * frame_height;

        rgb_params.clear();
        gray_params.clear();
        if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
            rgb_params.resize(pixels_count);
        else
            gray_params.resize(pixels_count);

        if (use_cuda)
        {
#ifdef USE_CUDA
            bool success_malloc = true;
            success_malloc &= h_frame_bgrxf.malloc(pixels_count);
            success_malloc &= d_bgr32.malloc(pixels_count);

            success_malloc &= d_curr_processes.malloc(pixels_count);
            success_malloc &= d_created_processes.malloc(pixels_count);
            success_malloc &= d_process1.malloc(pixels_count);
            success_malloc &= d_process2.malloc(pixels_count);
            success_malloc &= d_process3.malloc(pixels_count);

            if (!success_malloc)
            {
                use_cuda = false;
                use_cuda_ = use_cuda;
            }
#else
            assert(false);
#endif
        }

        init_filter = false;

        return true;
    }
    return false;
}
////////////////////////////////////////////////////////////////////////////

#ifndef ADV_OUT
#ifdef USE_CUDA
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask)
#else
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l)
#endif
#else
#ifdef USE_CUDA
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, uchar* adv_buf_rgb24)
#else
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, uchar* adv_buf_rgb24)
#endif
#endif
{
#ifndef ADV_OUT
#ifdef USE_CUDA
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, rgb_params);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, gray_params);
#else
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, rgb_params);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, gray_params);
#endif
#else
#ifdef USE_CUDA
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, rgb_params, adv_buf_rgb24);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, d_mask, gray_params, adv_buf_rgb24);
#else
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        return background_substraction(curr_frame, buf, pitch, pixels_l, rgb_params, adv_buf_rgb24);
    else
        return background_substraction(curr_frame, buf, pitch, pixels_l, gray_params, adv_buf_rgb24);
#endif
#endif
}
////////////////////////////////////////////////////////////////////////////

template<class PARAMS_CONT>
#ifndef ADV_OUT
#ifdef USE_CUDA
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params)
#else
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params)
#endif
#else
#ifdef USE_CUDA
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, CCudaBuf<mask_type, true>& d_mask, PARAMS_CONT& params, uchar* adv_buf_rgb24)
#else
int CGaussianMixtureBackSubstr::background_substraction(int& curr_frame, const uchar* buf, uint pitch, mask_cont& pixels_l, PARAMS_CONT& params, uchar* adv_buf_rgb24)
#endif
#endif
{
    // �������� � ������� 1-� �������: ��������� ���������� ������� � ������������������ ����������
    if (!init_filter && (curr_frame < fps))
    {
        ++curr_frame;

        auto* par = &params[0];
        const uchar* pbuf = buf;
        for (uint y = 0; y < frame_height; ++y)
        {
            for (uint x = 0; x < frame_width; ++x)
            {
                if (curr_frame == fps)
                    par->end_create_statistic(pbuf, (ft_float_t)curr_frame, min_sigma_val, max_sigma_val);
                else
                    par->create_statistic(pbuf, (ft_float_t)curr_frame);

                pbuf += PIXEL_SIZE;
                ++par;
            }
            pbuf += pitch - frame_width * PIXEL_SIZE;
        }

        if (curr_frame == fps)
        {
            if (use_cuda)
            {
#ifdef USE_CUDA
                // ������ ��� ������ �������������� ������
                switch (curr_color_type)
                {
                case buf_rgb24:
                case buf_rgb32:
                {
                    std::vector<long> h_curr_processes(params.size(), 0);
                    std::vector<long> h_created_processes(params.size(), 1);

                    std::vector<BgrndProcess> h_process1(params.size());
                    std::vector<BgrndProcess> h_process2(params.size());
                    std::vector<BgrndProcess> h_process3(params.size());

                    for (size_t i = 0, stop = params.size(); i < stop; ++i)
                    {
                        h_process1[i].mu[0] = params[i].proc_list[0].p[0].mu;
                        h_process1[i].sigma[0] = params[i].proc_list[0].p[0].sigma;
                        h_process1[i].mu[1] = params[i].proc_list[0].p[1].mu;
                        h_process1[i].sigma[1] = params[i].proc_list[0].p[1].sigma;
                        h_process1[i].mu[2] = params[i].proc_list[0].p[2].mu;
                        h_process1[i].sigma[2] = params[i].proc_list[0].p[2].sigma;
                        h_process1[i].weight = 1;
                    }

                    cudaMemcpy(d_curr_processes.buf, &h_curr_processes[0], d_curr_processes.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_created_processes.buf, &h_created_processes[0], d_created_processes.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_process1.buf, &h_process1[0], d_process1.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_process2.buf, &h_process2[0], d_process2.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_process3.buf, &h_process3[0], d_process3.buf_size, cudaMemcpyHostToDevice);
                    break;
                }

                case buf_gray:
                {
                    std::vector<long> h_curr_processes(params.size(), 0);
                    std::vector<long> h_created_processes(params.size(), 1);

                    std::vector<BgrndProcess> h_process1(params.size());
                    std::vector<BgrndProcess> h_process2(params.size());
                    std::vector<BgrndProcess> h_process3(params.size());

                    for (size_t i = 0, stop = params.size(); i < stop; ++i)
                    {
                        h_process1[i].mu[0] = params[i].proc_list[0].p[0].mu;
                        h_process1[i].sigma[0] = params[i].proc_list[0].p[0].sigma;
                        h_process1[i].weight = 1;
                    }

                    cudaMemcpy(d_curr_processes.buf, &h_curr_processes[0], d_curr_processes.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_created_processes.buf, &h_created_processes[0], d_created_processes.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_process1.buf, &h_process1[0], d_process1.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_process2.buf, &h_process2[0], d_process2.buf_size, cudaMemcpyHostToDevice);
                    cudaMemcpy(d_process3.buf, &h_process3[0], d_process3.buf_size, cudaMemcpyHostToDevice);
                    break;
                }
                }
#else
                assert(false);
#endif
            }

            curr_frame = 0;
            init_filter = true;
        }
        return 0;
    }

    auto* par = &params[0];
    const uchar* pbuf = buf;

    ft_float_t tmp_eps[3] = { epsilon, epsilon, epsilon };

    if (use_cuda)
    {
#ifdef USE_CUDA
        // ����������� ����� � ����������� �� 4 ����� ������
        switch (curr_color_type)
        {
        case buf_rgb24:
            copy_24to32((uchar*)h_frame_bgrxf.buf, 4 * frame_width, pbuf, frame_width, frame_height);
            break;

        case buf_gray:
            copy_gray_to_float((float*)h_frame_bgrxf.buf, pbuf, frame_width, frame_height);
            break;

        case buf_rgb32:
            memcpy((uchar*)h_frame_bgrxf.buf, pbuf, 4 * frame_width * frame_height);
            break;
        }

        // ����������� ����� � �����������
        if (cudaMemcpy(d_bgr32.buf, h_frame_bgrxf.buf, d_bgr32.buf_size, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            set_show_objects(false);
            return 0;
        }

        // ��������� ��������� ����
        PixelUpdateParams pup = { alpha1, alpha2, min_sigma_val, max_sigma_val };
        MixturePixelUpdateParams mup = { alpha3, weight_threshold };

        if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
            back_substraction_mixture(d_bgr32.buf, d_process1.buf, d_process2.buf, d_process3.buf, d_curr_processes.buf, d_created_processes.buf, d_mask.buf, frame_width, frame_height, tmp_eps[0], tmp_eps[1], tmp_eps[2], pup, mup);
        else
            back_substraction_mixture_gray((float *)d_bgr32.buf, d_process1.buf, d_process2.buf, d_process3.buf, d_curr_processes.buf, d_created_processes.buf, d_mask.buf, frame_width, frame_height, epsilon, pup, mup);
#else
        assert(false);
#endif
    }
    else
    {
#ifdef ADV_OUT
        uchar* adv_dest_buf = adv_buf_rgb24;
#endif

        ft_float_t float_src[3];

        // ������ ��������� ��������� ����
        size_t i = 0;
        for (uint y = 0; y < frame_height; ++y)
        {
            for (uint x = 0; x < frame_width; ++x, ++i)
            {
#ifdef ADV_OUT
                adv_dest_buf[0] = (uchar)par->proc_list[par->curr_proc].p[0].mu;
                if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
                {
                    adv_dest_buf[1] = (uchar)par->proc_list[par->curr_proc].p[1].mu;
                    adv_dest_buf[2] = (uchar)par->proc_list[par->curr_proc].p[2].mu;
                }
                else
                {
                    adv_dest_buf[1] = (uchar)par->proc_list[par->curr_proc].p[0].mu;
                    adv_dest_buf[2] = (uchar)par->proc_list[par->curr_proc].p[0].mu;
                }
#endif

                // ������������� �������
                float_src[0] = (ft_float_t)pbuf[0];
                if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
                {
                    float_src[1] = (ft_float_t)pbuf[1];
                    float_src[2] = (ft_float_t)pbuf[2];
                }

                if (par->is_back(float_src, tmp_eps, alpha1, alpha2, alpha3, min_sigma_val, max_sigma_val, weight_threshold) ||
                        is_patch_of_sunlight(float_src))
                {
                    // ������� ����������� ������� �����
                    pixels_l[i] = 0;
                }
                else
                {
                    // ������� ����������� ��������� �����
                    pixels_l[i] = 1;

#ifdef ADV_OUT // ������� ��� ������� ��������� ����� ������ ������
#if 1
                    adv_dest_buf[0] = 0;
                    adv_dest_buf[1] = 0;
                    adv_dest_buf[2] = 0xff;
#endif
#endif
                }
                pbuf += PIXEL_SIZE;
                ++par;
#ifdef ADV_OUT
                adv_dest_buf += 3;
#endif
            }
            pbuf += pitch - frame_width * PIXEL_SIZE;
        }
    }
    return 1;
}
////////////////////////////////////////////////////////////////////////////

void CGaussianMixtureBackSubstr::update_statistic_in_region(const uchar* /*buf*/, uint /*pitch*/, const CObjectRegion& /*region*/)
{
}
////////////////////////////////////////////////////////////////////////////

void CGaussianMixtureBackSubstr::reset_statistic_in_region(const uchar* buf, uint pitch, const CObjectRegion& region)
{
    if ((curr_color_type == buf_rgb24) || (curr_color_type == buf_rgb32))
        reset_statistic_in_region(buf, pitch, rgb_params, region);
    else
        reset_statistic_in_region(buf, pitch, gray_params, region);
}
////////////////////////////////////////////////////////////////////////////

template<class PARAMS_CONT>
void CGaussianMixtureBackSubstr::reset_statistic_in_region(const uchar* buf, uint pitch, PARAMS_CONT& params, const CObjectRegion& region)
{
    if (use_cuda)
    {
    }
    else
    {
        // ������������� ������������� ������� � �������� ������� ������� ����������� ����������� ��������
        int w1 = pitch - PIXEL_SIZE * region.width();
        int w2 = frame_width - region.width();

        buf += PIXEL_SIZE * region.get_left() + pitch * region.get_top();

        auto* par = &params[region.get_left() + frame_width * region.get_top()];

        for (int y = region.get_top(); y <= region.get_bottom(); ++y)
        {
            for (int x = region.get_left(); x <= region.get_right(); ++x)
            {
                par->set_mu_sigma(buf, max_sigma_val);

                ++par;
                buf += PIXEL_SIZE;
            }
            par += w2;
            buf += w1;
        }
    }
}
////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
