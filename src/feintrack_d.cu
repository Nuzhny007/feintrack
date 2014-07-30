#include "feintrack_d.h"
////////////////////////////////////////////////////////////////////////////////

const int MAX_THREADS_PER_BLOCK = 512;
const int BACK_BLOCK_SIZE = 16;
////////////////////////////////////////////////////////////////////////////////

template<class T>
__device__ T sqr_(T val)
{
	return val * val;
}
////////////////////////////////////////////////////////////////////////

#define CALC_I() gridDim.x * blockIdx.y * blockDim.x * blockDim.y + blockIdx.x * blockDim.x + threadIdx.y * gridDim.x * blockDim.x + threadIdx.x
////////////////////////////////////////////////////////////////////////

#define get_b(v) (unsigned char)(0x000000ff & (v));
#define get_g(v) (unsigned char)(0x000000ff & ((v) >> 8));
#define get_r(v) (unsigned char)(0x000000ff & ((v) >> 16));
////////////////////////////////////////////////////////////////////////

__global__ void back_substraction_(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, mask_type *mask, float eps_b, float eps_g, float eps_r)
{
	int i = CALC_I();
	int32_t p = bgr32[i];
	float b = (float)get_b(p);
	float g = (float)get_g(p);
	float r = (float)get_r(p);

	mask_type res = 1;

	if (eps_b * params_b_sigma[i] > fabsf(params_b_mu[i] - b) &&
		eps_g * params_g_sigma[i] > fabsf(params_g_mu[i] - g) &&
		eps_r * params_r_sigma[i] > fabsf(params_r_mu[i] - r))
	{
		res = 0;
	}
	mask[i] = res;
}
////////////////////////////////////////////////////////////////////////////////

void back_substraction(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, mask_type *mask,
					   int frame_width, int frame_height, float eps_b, float eps_g, float eps_r)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	back_substraction_<<<dim_grid, dim_block>>>(bgr32, params_b_mu, params_b_sigma, params_g_mu, params_g_sigma, params_r_mu, params_r_sigma, mask, eps_b, eps_g, eps_r);
}
////////////////////////////////////////////////////////////////////////////////

__global__ void back_substraction_gray_(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, float eps)
{
	int i = CALC_I();

	mask_type res = 1;

	if (eps * params_sigma[i] > fabsf(params_mu[i] - p_val[i]))
		res = 0;

	mask[i] = res;
}
////////////////////////////////////////////////////////////////////////////////

void back_substraction_gray(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, int frame_width, int frame_height, float eps)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	back_substraction_gray_<<<dim_grid, dim_block>>>(p_val, params_mu, params_sigma, mask, eps);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

__global__ void back_substraction_upd_(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma,
									   mask_type *mask, float eps_b, float eps_g, float eps_r, PixelUpdateParams pup)
{
	int i = CALC_I();

	int32_t p = bgr32[i];
	float b = (float)get_b(p);
	float g = (float)get_g(p);
	float r = (float)get_r(p);

	mask_type res = 1;

	float b_mu = params_b_mu[i];
	float b_sigma = params_b_sigma[i];
	float g_mu = params_g_mu[i];
	float g_sigma = params_g_sigma[i];
	float r_mu = params_r_mu[i];
	float r_sigma = params_r_sigma[i];

	if (eps_b * b_sigma > fabsf(b_mu - b) &&
		eps_g * g_sigma > fabsf(g_mu - g) &&
		eps_r * r_sigma > fabsf(r_mu - r))
	{
		res = 0;

		b_mu = (1.f - pup.alpha1) * b_mu + pup.alpha1 * b;
		g_mu = (1.f - pup.alpha1) * g_mu + pup.alpha1 * g;
		r_mu = (1.f - pup.alpha1) * r_mu + pup.alpha1 * r;

		b_sigma = sqrtf((1.f - pup.alpha2) * sqr_(b_sigma) + pup.alpha2 * sqr_(b - b_mu));
		g_sigma = sqrtf((1.f - pup.alpha2) * sqr_(g_sigma) + pup.alpha2 * sqr_(g - g_mu));
		r_sigma = sqrtf((1.f - pup.alpha2) * sqr_(r_sigma) + pup.alpha2 * sqr_(r - r_mu));

		b_sigma = fmaxf(fminf(pup.min_sigma_val, b_sigma), pup.max_sigma_val);
		g_sigma = fmaxf(fminf(pup.min_sigma_val, g_sigma), pup.max_sigma_val);
		r_sigma = fmaxf(fminf(pup.min_sigma_val, r_sigma), pup.max_sigma_val);

		params_b_mu[i] = b_mu;
		params_b_sigma[i] = b_sigma;
		params_g_mu[i] = g_mu;
		params_g_sigma[i] = g_sigma;
		params_r_mu[i] = r_mu;
		params_r_sigma[i] = r_sigma;
	}
	mask[i] = res;
}
////////////////////////////////////////////////////////////////////////////////

void back_substraction_upd(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma,
						   mask_type *mask, int frame_width, int frame_height, float eps_b, float eps_g, float eps_r, PixelUpdateParams pup)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	back_substraction_upd_<<<dim_grid, dim_block>>>(bgr32, params_b_mu, params_b_sigma, params_g_mu, params_g_sigma, params_r_mu, params_r_sigma, mask, eps_b, eps_g, eps_r, pup);
}
////////////////////////////////////////////////////////////////////////////////

__global__ void back_substraction_gray_upd_(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, float eps, PixelUpdateParams pup)
{
	int i = CALC_I();

	float p = p_val[i];

	mask_type res = 1;

	float mu = params_mu[i];
	float sigma = params_sigma[i];

	if (eps * sigma > fabsf(mu - p))
	{
		res = 0;

		mu = (1.f - pup.alpha1) * mu + pup.alpha1 * p;

		sigma = sqrtf((1.f - pup.alpha2) * sqr_(sigma) + pup.alpha2 * sqr_(p - mu));
		sigma = fmaxf(fminf(pup.min_sigma_val, sigma), pup.max_sigma_val);

		params_mu[i] = mu;
		params_sigma[i] = sigma;
	}
	mask[i] = res;
}
////////////////////////////////////////////////////////////////////////////////

void back_substraction_gray_upd(float *p_val, float *params_mu, float *params_sigma, mask_type *mask, int frame_width, int frame_height, float eps, PixelUpdateParams pup)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	back_substraction_gray_upd_<<<dim_grid, dim_block>>>(p_val, params_mu, params_sigma, mask, eps, pup);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

__global__ void back_substraction_mixture_(int32_t* bgr32,
										   BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
										   int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
										   float eps_b, float eps_g, float eps_r,
										   PixelUpdateParams pup, MixturePixelUpdateParams mup)
{
	int i = CALC_I();

	int32_t p = bgr32[i];
	float b = (float)get_b(p);
	float g = (float)get_g(p);
	float r = (float)get_r(p);

	mask_type res = 1;

	bool find_process = false;

	int32_t curr_proc = curr_processes[i];
	int32_t cr_processes = created_processes[i];
	BgrndProcess proc_list[3] = { process1[i], process2[i], process3[i] };

	for (size_t proc_ind = 0; proc_ind < cr_processes; ++proc_ind)
	{
		// Ищем процесс, который лучше соответствует текущему значению пикселя
		if (eps_b * proc_list[proc_ind].sigma[0] > fabsf(proc_list[proc_ind].mu[0] - b) &&
			eps_g * proc_list[proc_ind].sigma[1] > fabsf(proc_list[proc_ind].mu[1] - g) &&
			eps_r * proc_list[proc_ind].sigma[2] > fabsf(proc_list[proc_ind].mu[2] - r))
		{
			// Процесс найден - уточняем его параметры
			curr_proc = proc_ind;

			// Оценки мат. ожидания и дисперсии обновляются с помощью низкочастотного фильтра рекурсивного сглаживания
			proc_list[proc_ind].mu[0] = (1.f - pup.alpha1) * proc_list[proc_ind].mu[0] + pup.alpha1 * b;
			proc_list[proc_ind].mu[1] = (1.f - pup.alpha1) * proc_list[proc_ind].mu[1] + pup.alpha1 * g;
			proc_list[proc_ind].mu[2] = (1.f - pup.alpha1) * proc_list[proc_ind].mu[2] + pup.alpha1 * r;

			proc_list[proc_ind].sigma[0] = sqrtf((1.f - pup.alpha2) * sqr_(proc_list[proc_ind].sigma[0]) + pup.alpha2 * sqr_(b - proc_list[proc_ind].mu[0]));
			proc_list[proc_ind].sigma[1] = sqrtf((1.f - pup.alpha2) * sqr_(proc_list[proc_ind].sigma[1]) + pup.alpha2 * sqr_(g - proc_list[proc_ind].mu[1]));
			proc_list[proc_ind].sigma[2] = sqrtf((1.f - pup.alpha2) * sqr_(proc_list[proc_ind].sigma[2]) + pup.alpha2 * sqr_(r - proc_list[proc_ind].mu[2]));

			proc_list[proc_ind].sigma[0] = fmaxf(fminf(pup.min_sigma_val, proc_list[proc_ind].sigma[0]), pup.max_sigma_val);
			proc_list[proc_ind].sigma[1] = fmaxf(fminf(pup.min_sigma_val, proc_list[proc_ind].sigma[1]), pup.max_sigma_val);
			proc_list[proc_ind].sigma[2] = fmaxf(fminf(pup.min_sigma_val, proc_list[proc_ind].sigma[2]), pup.max_sigma_val);

			find_process = true;
			break;
		}
	}
	if (!find_process) // Процесс не найден
	{
		// Создаём новый процесс или,
		if (cr_processes < 3)
		{
			++cr_processes;
			curr_proc = cr_processes - 1;

			proc_list[curr_proc].mu[0] = b;
			proc_list[curr_proc].mu[1] = g;
			proc_list[curr_proc].mu[2] = r;

			proc_list[curr_proc].sigma[0] = pup.min_sigma_val;
			proc_list[curr_proc].sigma[1] = pup.min_sigma_val;
			proc_list[curr_proc].sigma[2] = pup.min_sigma_val;

			find_process = true;
		}
		// если количество процессов равно 3, ищем процесс с наименьшим весом
		else
		{
			float min_weight = proc_list[0].weight;
			size_t min_proc = 0;
			for (size_t proc_ind = 1; proc_ind < cr_processes; ++proc_ind)
			{
				if (proc_list[proc_ind].weight < min_weight)
				{
					min_proc = proc_ind;
					min_weight = proc_list[proc_ind].weight;
				}
			}
			curr_proc = min_proc;

			proc_list[curr_proc].mu[0] = b;
			proc_list[curr_proc].mu[1] = g;
			proc_list[curr_proc].mu[2] = r;

			proc_list[curr_proc].sigma[0] = pup.min_sigma_val;
			proc_list[curr_proc].sigma[1] = pup.min_sigma_val;
			proc_list[curr_proc].sigma[2] = pup.min_sigma_val;
		}
	}

	// Обновление весов процессов
	if (find_process)
	{
		for (size_t proc_ind = 0; proc_ind < cr_processes; ++proc_ind)
		{
			proc_list[proc_ind].weight = (1 - mup.alpha3) * proc_list[proc_ind].weight + mup.alpha3 * ((proc_ind == curr_proc) ? 1 : 0);
		}
	}

	if (proc_list[curr_proc].weight > mup.weight_threshold)
		res = 0;

	mask[i] = res;
	curr_processes[i] = curr_proc;
	created_processes[i] = cr_processes;

	process1[i] = proc_list[0];
	process2[i] = proc_list[1];
	process3[i] = proc_list[2];
}
////////////////////////////////////////////////////////////////////////////////

void back_substraction_mixture(int32_t* bgr32, BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
							   int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
							   int frame_width, int frame_height, float eps_b, float eps_g, float eps_r,
							   PixelUpdateParams pup, MixturePixelUpdateParams mup)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	back_substraction_mixture_<<<dim_grid, dim_block>>>(bgr32, process1, process2, process3, curr_processes, created_processes, mask, eps_b, eps_g, eps_r, pup, mup);
}
////////////////////////////////////////////////////////////////////////////////

__global__ void back_substraction_mixture_gray_(float* p_val,
												BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
												int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
												float eps, PixelUpdateParams pup, MixturePixelUpdateParams mup)
{
	int i = CALC_I();

	float p = p_val[i];

	mask_type res = 1;

	bool find_process = false;

	int32_t curr_proc = curr_processes[i];
	int32_t cr_processes = created_processes[i];
	BgrndProcess proc_list[3] = { process1[i], process2[i], process3[i] };

	for (size_t proc_ind = 0; proc_ind < cr_processes; ++proc_ind)
	{
		// Ищем процесс, который лучше соответствует текущему значению пикселя
		if (eps * proc_list[proc_ind].sigma[0] > fabsf(proc_list[proc_ind].mu[0] - p))
		{
			// Процесс найден - уточняем его параметры
			curr_proc = proc_ind;

			// Оценки мат. ожидания и дисперсии обновляются с помощью низкочастотного фильтра рекурсивного сглаживания
			proc_list[proc_ind].mu[0] = (1.f - pup.alpha1) * proc_list[proc_ind].mu[0] + pup.alpha1 * p;
			proc_list[proc_ind].sigma[0] = sqrtf((1.f - pup.alpha2) * sqr_(proc_list[proc_ind].sigma[0]) + pup.alpha2 * sqr_(p - proc_list[proc_ind].mu[0]));
			proc_list[proc_ind].sigma[0] = fmaxf(fminf(pup.min_sigma_val, proc_list[proc_ind].sigma[0]), pup.max_sigma_val);

			find_process = true;
			break;
		}
	}
	if (!find_process) // Процесс не найден
	{
		// Создаём новый процесс или,
		if (cr_processes < 3)
		{
			++cr_processes;
			curr_proc = cr_processes - 1;

			proc_list[curr_proc].mu[0] = p;
			proc_list[curr_proc].sigma[0] = pup.min_sigma_val;

			find_process = true;
		}
		// если количество процессов равно 3, ищем процесс с наименьшим весом
		else
		{
			float min_weight = proc_list[0].weight;
			size_t min_proc = 0;
			for (size_t proc_ind = 1; proc_ind < cr_processes; ++proc_ind)
			{
				if (proc_list[proc_ind].weight < min_weight)
				{
					min_proc = proc_ind;
					min_weight = proc_list[proc_ind].weight;
				}
			}
			curr_proc = min_proc;

			proc_list[curr_proc].mu[0] = p;
			proc_list[curr_proc].sigma[0] = pup.min_sigma_val;
		}
	}

	// Обновление весов процессов
	if (find_process)
	{
		for (size_t proc_ind = 0; proc_ind < cr_processes; ++proc_ind)
		{
			proc_list[proc_ind].weight = (1 - mup.alpha3) * proc_list[proc_ind].weight + mup.alpha3 * ((proc_ind == curr_proc) ? 1 : 0);
		}
	}

	if (proc_list[curr_proc].weight > mup.weight_threshold)
		res = 0;

	mask[i] = res;
	curr_processes[i] = curr_proc;
	created_processes[i] = cr_processes;

	process1[i] = proc_list[0];
	process2[i] = proc_list[1];
	process3[i] = proc_list[2];
}
////////////////////////////////////////////////////////////////////////////////

void back_substraction_mixture_gray(float* p_val, BgrndProcess* process1, BgrndProcess* process2, BgrndProcess* process3,
							   int32_t* curr_processes, int32_t* created_processes, mask_type* mask,
							   int frame_width, int frame_height, float eps,
							   PixelUpdateParams pup, MixturePixelUpdateParams mup)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	back_substraction_mixture_gray_<<<dim_grid, dim_block>>>(p_val, process1, process2, process3, curr_processes, created_processes, mask, eps, pup, mup);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

__global__ void reset_statistic_(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
								 int left, int right, int top, int bottom, float max_sigma_val)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	int reset_pixels = (right - left + 1) * (bottom - top + 1);

	if (i < reset_pixels)
	{
		i += left + frame_width * top;

		int32_t p = bgr32[i];
		float b = (float)get_b(p);
		float g = (float)get_g(p);
		float r = (float)get_r(p);

		params_b_mu[i] = b;
		params_g_mu[i] = g;
		params_r_mu[i] = r;

		params_b_sigma[i] = max_sigma_val;
		params_g_sigma[i] = max_sigma_val;
		params_r_sigma[i] = max_sigma_val;
	}
}
////////////////////////////////////////////////////////////////////////////////

void reset_statistic(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
					 int left, int right, int top, int bottom, float max_sigma_val)
{
	int reset_pixels = (right - left + 1) * (bottom - top + 1);
	reset_pixels += MAX_THREADS_PER_BLOCK - reset_pixels % MAX_THREADS_PER_BLOCK;

	dim3 dim_block = dim3(MAX_THREADS_PER_BLOCK, 1);
	dim3 dim_grid = dim3(reset_pixels / dim_block.x, 1);

	reset_statistic_<<<dim_grid, dim_block>>>(bgr32, params_b_mu, params_b_sigma, params_g_mu, params_g_sigma, params_r_mu, params_r_sigma, frame_width, left, right, top, bottom, max_sigma_val);
}
////////////////////////////////////////////////////////////////////////////////

__global__ void reset_statistic_gray_(float *p_val, float *params_mu, float *params_sigma, int frame_width,
								 int left, int right, int top, int bottom, float max_sigma_val)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	int reset_pixels = (right - left + 1) * (bottom - top + 1);

	if (i < reset_pixels)
	{
		i += left + frame_width * top;

		params_mu[i] = p_val[i];
		params_sigma[i] = max_sigma_val;
	}
}
////////////////////////////////////////////////////////////////////////////////

void reset_statistic_gray(float *p_val, float *params_mu, float *params_sigma, int frame_width,
					 int left, int right, int top, int bottom, float max_sigma_val)
{
	int reset_pixels = (right - left + 1) * (bottom - top + 1);
	reset_pixels += MAX_THREADS_PER_BLOCK - reset_pixels % MAX_THREADS_PER_BLOCK;

	dim3 dim_block = dim3(MAX_THREADS_PER_BLOCK, 1);
	dim3 dim_grid = dim3(reset_pixels / dim_block.x, 1);

	reset_statistic_gray_<<<dim_grid, dim_block>>>(p_val, params_mu, params_sigma, frame_width, left, right, top, bottom, max_sigma_val);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

__global__ void update_statistic_(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
								  int left, int right, int top, int bottom, PixelUpdateParams pup)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	int reset_pixels = (right - left + 1) * (bottom - top + 1);

	if (i < reset_pixels)
	{
		i += left + frame_width * top;

		float b_mu = params_b_mu[i];
		float b_sigma = params_b_sigma[i];
		float g_mu = params_g_mu[i];
		float g_sigma = params_g_sigma[i];
		float r_mu = params_r_mu[i];
		float r_sigma = params_r_sigma[i];

		int32_t p = bgr32[i];
		float b = (float)get_b(p);
		float g = (float)get_g(p);
		float r = (float)get_r(p);

		b_mu = (1.f - pup.alpha1) * b_mu + pup.alpha1 * b;
		g_mu = (1.f - pup.alpha1) * g_mu + pup.alpha1 * g;
		r_mu = (1.f - pup.alpha1) * r_mu + pup.alpha1 * r;

		b_sigma = sqrtf((1.f - pup.alpha2) * sqr_(b_sigma) + pup.alpha2 * sqr_(b - b_mu));
		g_sigma = sqrtf((1.f - pup.alpha2) * sqr_(g_sigma) + pup.alpha2 * sqr_(g - g_mu));
		r_sigma = sqrtf((1.f - pup.alpha2) * sqr_(r_sigma) + pup.alpha2 * sqr_(r - r_mu));

		b_sigma = fmaxf(fminf(pup.min_sigma_val, b_sigma), pup.max_sigma_val);
		g_sigma = fmaxf(fminf(pup.min_sigma_val, g_sigma), pup.max_sigma_val);
		r_sigma = fmaxf(fminf(pup.min_sigma_val, r_sigma), pup.max_sigma_val);

		params_b_mu[i] = b_mu;
		params_b_sigma[i] = b_sigma;
		params_g_mu[i] = g_mu;
		params_g_sigma[i] = g_sigma;
		params_r_mu[i] = r_mu;
		params_r_sigma[i] = r_sigma;
	}
}
////////////////////////////////////////////////////////////////////////////////

void update_statistic(int32_t *bgr32, float *params_b_mu, float *params_b_sigma, float *params_g_mu, float *params_g_sigma, float *params_r_mu, float *params_r_sigma, int frame_width,
					  int left, int right, int top, int bottom, PixelUpdateParams pup)
{
	int reset_pixels = (right - left + 1) * (bottom - top + 1);
	reset_pixels += MAX_THREADS_PER_BLOCK - reset_pixels % MAX_THREADS_PER_BLOCK;

	dim3 dim_block = dim3(MAX_THREADS_PER_BLOCK, 1);
	dim3 dim_grid = dim3(reset_pixels / dim_block.x, 1);

	update_statistic_<<<dim_grid, dim_block>>>(bgr32, params_b_mu, params_b_sigma, params_g_mu, params_g_sigma, params_r_mu, params_r_sigma, frame_width, left, right, top, bottom, pup);
}
////////////////////////////////////////////////////////////////////////////////

__global__ void update_statistic_gray_(float *p_val, float *params_mu, float *params_sigma, int frame_width,
								  int left, int right, int top, int bottom, PixelUpdateParams pup)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	int reset_pixels = (right - left + 1) * (bottom - top + 1);

	if (i < reset_pixels)
	{
		i += left + frame_width * top;

		float mu = params_mu[i];
		float sigma = params_sigma[i];

		float p = p_val[i];

		mu = (1.f - pup.alpha1) * mu + pup.alpha1 * p;

		sigma = sqrtf((1.f - pup.alpha2) * sqr_(sigma) + pup.alpha2 * sqr_(p - mu));
		sigma = fmaxf(fminf(pup.min_sigma_val, sigma), pup.max_sigma_val);

		params_mu[i] = mu;
		params_sigma[i] = sigma;
	}
}
////////////////////////////////////////////////////////////////////////////////

void update_statistic_gray(float *p_val, float *params_mu, float *params_sigma, int frame_width,
					  int left, int right, int top, int bottom, PixelUpdateParams pup)
{
	int reset_pixels = (right - left + 1) * (bottom - top + 1);
	reset_pixels += MAX_THREADS_PER_BLOCK - reset_pixels % MAX_THREADS_PER_BLOCK;

	dim3 dim_block = dim3(MAX_THREADS_PER_BLOCK, 1);
	dim3 dim_grid = dim3(reset_pixels / dim_block.x, 1);

	update_statistic_gray_<<<dim_grid, dim_block>>>(p_val, params_mu, params_sigma, frame_width, left, right, top, bottom, pup);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Структурный элемент - прямоугольник 3х3
__global__ void morphology_(mask_type *mask, mask_type *mask_temp, int frame_width, int frame_height, unsigned int pixels)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;

	if ((i < frame_width) || (i >= pixels - frame_width) || (i % frame_width == 0) || (i % (frame_width - 1) == 0))
	{
		mask_temp[i] = 0;
	}
	else
	{
		// Эрозия
		mask_temp[i] = mask[i - frame_width - 1] & mask[i - frame_width] & mask[i - frame_width + 1] &
			mask[i - 1] & mask[i] & mask[i + 1] &
			mask[i + frame_width - 1] & mask[i + frame_width] & mask[i + frame_width + 1];

		__syncthreads();
		// Наращивание
		if (mask_temp[i])
		{
			mask[i - frame_width - 1] = 1;
			mask[i - frame_width] = 1;
			mask[i - frame_width + 1] = 1;
			mask[i - 1] = 1;
			mask[i] = 1;
			mask[i + 1] = 1;
			mask[i + frame_width - 1] = 1;
			mask[i + frame_width] = 1;
			mask[i + frame_width + 1] = 1;
		}
		else
		{
			mask[i] = 0;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////

void morphology(mask_type *mask, mask_type *mask_temp, int frame_width, int frame_height, unsigned int pixels)
{
	dim3 dim_block = dim3(MAX_THREADS_PER_BLOCK, 1);
	dim3 dim_grid = dim3(pixels / dim_block.x, 1);
	morphology_<<<dim_grid, dim_block>>>(mask, mask_temp, frame_width, frame_height, pixels);
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#if 1
__global__ void segmentation_(mask_type *mask, reg_label *regions)
{
	int tx = threadIdx.x;
	int ty = threadIdx.y;

	int mi = gridDim.x * blockIdx.y * blockDim.x * blockDim.y + blockIdx.x * blockDim.x + ty * gridDim.x * blockDim.x + tx;

	__shared__ mask_type data[SEGM_BLOCK_SIZE][SEGM_BLOCK_SIZE];
	data[tx][ty] = mask[mi];
	__syncthreads();

	if ((tx == 0) && (ty == 0))
	{
		mask_type s = 0;
		for (int i = 0; i < SEGM_BLOCK_SIZE; ++i)
		{
			for (int j = 0; j < SEGM_BLOCK_SIZE; ++j)
			{
				s += data[j][i];
			}
		}
		int bi = blockIdx.x + blockIdx.y * gridDim.x;
		regions[bi] = (reg_label)s;
	}
}
////////////////////////////////////////////////////////////////////////////////
void segmentation(mask_type *mask, reg_label *regions, int frame_width, int frame_height)
{
	dim3 dim_block = dim3(SEGM_BLOCK_SIZE, SEGM_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	segmentation_<<<dim_grid, dim_block>>>(mask, regions);
}
#else
__global__ void segmentation_(mask_type *mask, reg_label *regions)
{
	int tx = threadIdx.x;
	int ty = threadIdx.y;

	int mi = CALC_I();

	__shared__ mask_type data1[BACK_BLOCK_SIZE][BACK_BLOCK_SIZE];
	data1[tx][ty] = mask[mi];
	__syncthreads();


	__shared__ mask_type data2[SEGM_BLOCK_SIZE][SEGM_BLOCK_SIZE];
	if (!tx && !ty)
	{
		for (int y = 0; y < SEGM_BLOCK_SIZE; ++y)
		{
			for (int x = 0; x < SEGM_BLOCK_SIZE; ++x)
			{
				data2[x][y] = 0;
			}
		}
	}
	__syncthreads();


	if (!tx)
	{
		int y = ty / SEGM_BLOCK_SIZE;
		for (int x = 0; x < BACK_BLOCK_SIZE; ++x)
		{
			data2[x / SEGM_BLOCK_SIZE][y] += data1[x][ty];
		}
	}
	__syncthreads();


	if (((tx % SEGM_BLOCK_SIZE) == 0) || ((ty % SEGM_BLOCK_SIZE) == 0))
	{
		const int rx = tx / SEGM_BLOCK_SIZE;
		const int ry = ty / SEGM_BLOCK_SIZE;
		const int xk = BACK_BLOCK_SIZE / SEGM_BLOCK_SIZE;
		const int yk = BACK_BLOCK_SIZE / SEGM_BLOCK_SIZE;
		int bi = xk * gridDim.x * blockIdx.y * yk + blockIdx.x * xk + ry * gridDim.x * xk + rx;
		regions[bi] = (reg_label)data2[rx][ry];
	}
}
////////////////////////////////////////////////////////////////////////////////
void segmentation(mask_type *mask, reg_label *regions, int frame_width, int frame_height)
{
	dim3 dim_block = dim3(BACK_BLOCK_SIZE, BACK_BLOCK_SIZE);
	dim3 dim_grid = dim3(frame_width / dim_block.x, frame_height / dim_block.y);
	segmentation_<<<dim_grid, dim_block>>>(mask, regions);
}
#endif
////////////////////////////////////////////////////////////////////////////////
