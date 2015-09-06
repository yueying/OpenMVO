/*******************************************************************************
 * 文件： link_pragmas.h
 * 时间： 2014/11/13 21:55
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 用于定义库的导入导出
 *
********************************************************************************/
#ifndef OPENMVO_BASE_LINK_PRAGMAS_H_
#define OPENMVO_BASE_LINK_PRAGMAS_H_

#include <openmvo/config.h>
#include <openmvo/utils/boost_join.h>

// ** 这边很重要! **
// 在每一个OPENMVO库中，我们要找到下列进行替换为当前项目:
//  OPENMVO_XXX_EXPORT, OPENMVO_XXX_IMPORT
//  BASE_IMPEXP, OPENMVO_xxx_EXPORTS

// 通过下面宏的定义，我们可以对编译好的lib直接引用，不需要在链接器中添加名称:
#if !defined(OPENMVO_BASE_EXPORTS) && (defined(_MSC_VER))
#	if defined(_DEBUG)
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("openmvo_base",OPENMVO_VERSION_POSTFIX),"d.lib"))
#	else
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("openmvo_base",OPENMVO_VERSION_POSTFIX),".lib"))
#	endif
#endif

/* 定义dll的导入导出
*/
#if defined(OPENMVO_OS_WINDOWS)

#    if defined(_MSC_VER) 
#        define OPENMVO_BASE_EXPORT __declspec(dllexport)
#        define OPENMVO_BASE_IMPORT __declspec(dllimport)
#    else /* 编译器不支持__declspec() */
#        define OPENMVO_BASE_EXPORT
#        define OPENMVO_BASE_IMPORT
#    endif
#    endif

/* 如果没有定义导出，则不管这个宏命令 */
#ifndef OPENMVO_BASE_EXPORT
#    define OPENMVO_BASE_EXPORT
#    define OPENMVO_BASE_IMPORT
#endif

/* 通过宏BASE_IMPEXP 确定编译成dll，以及使用dll，或者不标识  */
#if defined(OPENMVO_BUILT_AS_DLL)
#	if defined(OPENMVO_BASE_EXPORTS)  /* 编译成dll */
#		define BASE_IMPEXP OPENMVO_BASE_EXPORT
#	else  /* 使用dll */
#		define BASE_IMPEXP OPENMVO_BASE_IMPORT
#	endif
#else /* 没有定义 */
#    define BASE_IMPEXP
#endif


#endif // OPENMVO_BASE_LINK_PRAGMAS_H_