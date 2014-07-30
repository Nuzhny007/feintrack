#pragma once

////////////////////////////////////////////////////////////////////////////////

typedef int32_t mask_type;
////////////////////////////////////////////////////////////////////////////////

// Выровненная структура, соответствующая цвету пикселя
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

// Параметры обновления статистической модели
typedef struct
{
	// Параметры обучения низкочастотного фильтра рекурсивного сглаживания
	float alpha1;           // Для выборочного среднего
	float alpha2;           // Для среднеквадратичного отклонения
	float min_sigma_val;    // Минимальное и
	float max_sigma_val;    // максимальное значение для среднеквадратичного отклонения (используется при вычитании фона)
} PixelUpdateParams;

////////////////////////////////////////////////////////////////////////////////

// Параметры обновления процессов для смеси гауссианов
typedef struct
{
	float alpha3;           // Параметр обучения низкочастотного фильтра рекурсивного сглаживания для веса процесса
	float weight_threshold; // Порог для веса процесса
} MixturePixelUpdateParams;

////////////////////////////////////////////////////////////////////////////////

// Параметры обновления статистической модели
typedef struct
{
	float mu[3];            // Выборочное среднее для каналов
	float sigma[3];         // Среднеквадратичное отклонение для каналов

	float weight;           // Вес процесса

	float reserved;         // Зарезервировано для выравнивания по 32 байта

} BgrndProcess;
////////////////////////////////////////////////////////////////////////////////

// Вычитание фона без обновления заднего плана
void back_substraction(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma,
					   mask_type *mask, int frame_width, int frame_height, float eps_b, float eps_g, float eps_r);

void back_substraction_gray(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, int frame_width, int frame_height, float eps);

////////////////////////////////////////////////////////////////////////////////

// Вычитание фона с обновлением заднего плана
void back_substraction_upd(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma,
						   mask_type *mask, int frame_width, int frame_height, float eps_b, float eps_g, float eps_r, PixelUpdateParams pup);

void back_substraction_gray_upd(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, int frame_width, int frame_height, float eps, PixelUpdateParams pup);
////////////////////////////////////////////////////////////////////////////////

// Вычитание фона с помощью смеси гауссианов
void back_substraction_mixture(int32_t* bgr32, BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
                               int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
							   int frame_width, int frame_height, float eps_b, float eps_g, float eps_r,
							   PixelUpdateParams pup, MixturePixelUpdateParams mup);

// Вычитание фона с помощью смеси гауссианов
void back_substraction_mixture_gray(float* p_val, BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
                                    int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
									int frame_width, int frame_height, float eps,
									PixelUpdateParams pup, MixturePixelUpdateParams mup);
////////////////////////////////////////////////////////////////////////////////

// Сброс статистики в указанной области
void reset_statistic(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
					 int left, int right, int top, int bottom, float max_sigma_val);

void reset_statistic_gray(float *p_val, float *params_mu, float *params_sigma, int frame_width,
					 int left, int right, int top, int bottom, float max_sigma_val);
////////////////////////////////////////////////////////////////////////////////

// Обновление статистики в указанной области
void update_statistic(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
					  int left, int right, int top, int bottom, PixelUpdateParams pup);

void update_statistic_gray(float *p_val, float *params_mu, float *params_sigma, int frame_width,
					  int left, int right, int top, int bottom, PixelUpdateParams pup);
////////////////////////////////////////////////////////////////////////////////

// Операция математической морфологии "открытие" для результатов вычитания фона
void morphology(mask_type *mask, mask_type *mask_temp, int frame_width, int frame_height, unsigned int pixels);
////////////////////////////////////////////////////////////////////////////////

// Предварительная поблочная сегментация объектов переднего плана
const int SEGM_BLOCK_SIZE = 4;
typedef int reg_label;
void segmentation(mask_type *mask, reg_label *regions, int frame_width, int frame_height);
////////////////////////////////////////////////////////////////////////////////
