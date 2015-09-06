/*******************************************************************************
 * 文件： noncopyable.h
 * 时间： 2014/11/05 23:57
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 不能复制类的基类，有任何的复制操作，编译器会报错.继承这个类就不能有复制操作
 *        主要用于单例等模式，具体可以参考boost::noncopyable
 *
 *
 ********************************************************************************/
#ifndef OPENMVO_BASE_NONCOPYABLE_H_
#define OPENMVO_BASE_NONCOPYABLE_H_

#include <openmvo/base/link_pragmas.h>

namespace mvo
{
	/** 不能复制类的基类，有任何的复制操作，编译器会报错.将深浅复制都写入父类的私有方法中，这样派生的子类也无法实现深浅复制
	 *  例子：
	 *
	 *  \code
	 *   class MyFancyClass : public openmvo::utils::CNoncopyable
	 *   {
	 *    public:
	 *     ...
	 *   };
	 *  \endcode
	 * \ingroup openmvo_base_grp
	 */
	class BASE_IMPEXP Noncopyable
	{
	protected:
		Noncopyable() {}
		~Noncopyable() {}
	private:
		Noncopyable(const Noncopyable &);  // 这不需要再其它地方实现
		Noncopyable& operator =(const Noncopyable &);   // 这不需要再其它地方实现
	}; // End of class def.

} // end of namespace

#endif // OPENMVO_BASE_NONCOPYABLE_H_
