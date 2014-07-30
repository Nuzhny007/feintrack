#pragma once

////////////////////////////////////////////////////////////////////////////////

typedef int32_t mask_type;
////////////////////////////////////////////////////////////////////////////////

// ����������� ���������, ��������������� ����� �������
#ifdef USE_GPU
typedef struct __align__(4)
{
    unsigned char b;
	unsigned char g;
	unsigned char r;
	unsigned char x;
} BGRXf;
#else
typedef struct
{
    unsigned char b;
    unsigned char g;
    unsigned char r;
    unsigned char x;
} BGRXf;
#endif

////////////////////////////////////////////////////////////////////////////////

// ��������� ���������� �������������� ������
typedef struct
{
	// ��������� �������� ��������������� ������� ������������ �����������
	float alpha1;           // ��� ����������� ��������
	float alpha2;           // ��� ������������������� ����������
	float min_sigma_val;    // ����������� �
	float max_sigma_val;    // ������������ �������� ��� ������������������� ���������� (������������ ��� ��������� ����)
} PixelUpdateParams;

////////////////////////////////////////////////////////////////////////////////

// ��������� ���������� ��������� ��� ����� ����������
typedef struct
{
	float alpha3;           // �������� �������� ��������������� ������� ������������ ����������� ��� ���� ��������
	float weight_threshold; // ����� ��� ���� ��������
} MixturePixelUpdateParams;

////////////////////////////////////////////////////////////////////////////////

// ��������� ���������� �������������� ������
typedef struct
{
	float mu[3];            // ���������� ������� ��� �������
	float sigma[3];         // ������������������ ���������� ��� �������

	float weight;           // ��� ��������

	float reserved;         // ��������������� ��� ������������ �� 32 �����

} BgrndProcess;
////////////////////////////////////////////////////////////////////////////////

// ��������� ���� ��� ���������� ������� �����
void back_substraction(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma,
					   mask_type *mask, int frame_width, int frame_height, float eps_b, float eps_g, float eps_r);

void back_substraction_gray(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, int frame_width, int frame_height, float eps);

////////////////////////////////////////////////////////////////////////////////

// ��������� ���� � ����������� ������� �����
void back_substraction_upd(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma,
						   mask_type *mask, int frame_width, int frame_height, float eps_b, float eps_g, float eps_r, PixelUpdateParams pup);

void back_substraction_gray_upd(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, int frame_width, int frame_height, float eps, PixelUpdateParams pup);
////////////////////////////////////////////////////////////////////////////////

// ��������� ���� � ������� ����� ����������
void back_substraction_mixture(int32_t* bgr32, BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
                               int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
							   int frame_width, int frame_height, float eps_b, float eps_g, float eps_r,
							   PixelUpdateParams pup, MixturePixelUpdateParams mup);

// ��������� ���� � ������� ����� ����������
void back_substraction_mixture_gray(float* p_val, BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
                                    int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
									int frame_width, int frame_height, float eps,
									PixelUpdateParams pup, MixturePixelUpdateParams mup);
////////////////////////////////////////////////////////////////////////////////

// ����� ���������� � ��������� �������
void reset_statistic(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
					 int left, int right, int top, int bottom, float max_sigma_val);

void reset_statistic_gray(float *p_val, float *params_mu, float *params_sigma, int frame_width,
					 int left, int right, int top, int bottom, float max_sigma_val);
////////////////////////////////////////////////////////////////////////////////

// ���������� ���������� � ��������� �������
void update_statistic(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
					  int left, int right, int top, int bottom, PixelUpdateParams pup);

void update_statistic_gray(float *p_val, float *params_mu, float *params_sigma, int frame_width,
					  int left, int right, int top, int bottom, PixelUpdateParams pup);
////////////////////////////////////////////////////////////////////////////////

// �������� �������������� ���������� "��������" ��� ����������� ��������� ����
void morphology(mask_type *mask, mask_type *mask_temp, int frame_width, int frame_height, unsigned int pixels);
////////////////////////////////////////////////////////////////////////////////

// ��������������� ��������� ����������� �������� ��������� �����
const int SEGM_BLOCK_SIZE = 4;
typedef int reg_label;
void segmentation(mask_type *mask, reg_label *regions, int frame_width, int frame_height);
////////////////////////////////////////////////////////////////////////////////
