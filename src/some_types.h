#pragma once

#ifdef USE_GPU
#include <cuda.h>
#include <cuda_runtime.h>
#endif

#include "feintrack_d.h"
////////////////////////////////////////////////////////////////////////////

namespace vl_feintrack
{
	////////////////////////////////////////////////////////////////////////////
	typedef std::vector<mask_type> mask_cont; // ��� ���������� ��� �������� ���������� ��������� ����

	////////////////////////////////////////////////////////////////////////////
#ifdef USE_GPU
	// ��������� ���� �� ���������
	template<int v>
	struct Int2Type
	{
		enum { value = v };
	};
	////////////////////////////////////////////////////////////////////////////

	// ������ ��� ���������� � ������� CUDA �������
	template <class T, bool DEVICE_MEM>
	struct CCudaBuf
	{
		T* buf;                          // ���������� ����� ������
		size_t buf_size;                 // ������ ������ � ������


        typedef cudaError_t (/*stdcall*/ MALLOC_FUNCTION)(void**, size_t);
        typedef cudaError_t (/*stdcall*/ FREE_FUNCTION)(void*);

        MALLOC_FUNCTION* malloc_func(Int2Type<true>)
		{
			return cudaMalloc;
		}

		FREE_FUNCTION* free_func(Int2Type<true>)
		{
			return cudaFree;
		}

		MALLOC_FUNCTION* malloc_func(Int2Type<false>)
		{
			return cudaMallocHost;
		}

		FREE_FUNCTION* free_func(Int2Type<false>)
		{
			return cudaFreeHost;
		}

		MALLOC_FUNCTION* malloc_func()   // ������� ��� ��������� �
		{
			return malloc_func(Int2Type<DEVICE_MEM>());
		}

		FREE_FUNCTION* free_func()       // ������������ ������
		{
			return free_func(Int2Type<DEVICE_MEM>());
		}

		CCudaBuf()
            : buf(nullptr), buf_size(0)
		{
		}

		CCudaBuf(size_t elements)
            : buf(nullptr), buf_size(0)
		{
			malloc(elements);
		}

		~CCudaBuf()
		{
			free();
		}

		void free()                      // �������� �����
		{
			if (buf)
			{
				free_func()(buf);
                buf = nullptr;
				buf_size = 0;
			}
		}

		bool malloc(size_t elements)     // �������� ������ ��� ���������� ����� ���������
		{
			size_t bytes = elements * sizeof(T);
#if 0       // ������ ����������� ������, ��� ��� ��� ����� �����-�� ����� �����������
			if (bytes != buf_size)
				free();
#else
			free();
#endif
			buf_size = bytes;
			return malloc_func()((void**)&buf, buf_size) == cudaSuccess;
		}
    };
#endif
	////////////////////////////////////////////////////////////////////////////
} //end namespace vl_feintrack
////////////////////////////////////////////////////////////////////////////
