/*************************************************************************
 * 文件名： aligned_mem
 *
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 * 时间： 2015/8/6
 *
 * 说明： 参考rpg_svo
 *************************************************************************/
#ifndef OPENMVO_UTILS_ALIGNED_MEM_H_
#define OPENMVO_UTILS_ALIGNED_MEM_H_

#include <stdlib.h>
#include <string>       // memset
#include <string.h>     // memset
#include <cassert>
#include <cstdlib>

namespace mvo {

	/// 检测指针的是否与指定字节对齐
	inline bool is_aligned8(const void* ptr)
	{
		return ((reinterpret_cast<size_t>(ptr)) & 0x7) == 0;
	}

	inline bool is_aligned16(const void* ptr)
	{
		return ((reinterpret_cast<size_t>(ptr)) & 0xF) == 0;
	}

	template <class T, int N = 20>
	struct placement_delete
	{
		enum { Size = (1 << N) };

		struct Array {
			T data[Size];
		};

		static inline void destruct(T* buf)
		{
			(*(Array*)buf).~Array();
		}

		static inline void free(T* buf, size_t M)
		{
			if (M >= Size) {
				placement_delete<T, N>::free(buf + Size, M - Size);
				placement_delete<T, N>::destruct(buf);
			}
			else {
				placement_delete<T, N - 1>::free(buf, M);
			}
		}
	};

	template <class T>
	struct placement_delete < T, -1 >
	{
		static inline void free(T*, size_t) {}
	};

	inline void * aligned_alloc(size_t count, size_t alignment){
		/*void * mem = NULL;
		assert(posix_memalign(&mem, alignment, count) == 0);
		return mem;*/
		return _aligned_malloc(count, alignment);
	}

	inline void aligned_free(void * memory) {
		free(memory);
	}

	template <class T>
	inline T * aligned_alloc(size_t count, size_t alignment){
		void * data = aligned_alloc(sizeof(T)* count, alignment);
		return new (data)T[count];
	}

	template <class T>
	inline void aligned_free(T * memory, size_t count){
		placement_delete<T>::free(memory, count);
		aligned_free(memory);
	}

	template<class T>
	inline void memfill(T* data, int n, const T val)
	{
		T* de = data + n;
		for (; data < de; data++)
			*data = val;
	}

	template<>
	inline void memfill(unsigned char* data, int n, const unsigned char val)
	{
		memset(data, val, n);
	}

	template<>
	inline void memfill(signed char* data, int n, const signed char val)
	{
		memset(data, val, n);
	}

	template<>
	inline void memfill(char* data, int n, const char val)
	{
		memset(data, val, n);
	}

	template <class T, int N>
	struct AlignedMem {
		T* mem;
		size_t count;
		AlignedMem(size_t c) : count(c) {
			mem = aligned_alloc<T>(count, N);
		}
		~AlignedMem() {
			aligned_free<T>(mem, count);
		}
		T* data() { return mem; }
		const T* data() const { return mem; }
	};

} // namespace mvo

#endif // OPENMVO_UTILS_ALIGNED_MEM_H_
