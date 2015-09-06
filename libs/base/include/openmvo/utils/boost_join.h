/*******************************************************************************
 * 文件： boost_join.h
 * 时间： 2014/11/21 22:55
 * 作者： 冯兵
 * 邮件： fengbing123@gmail.com
 *
 * 说明： 构建类似boost的名称的dll
 *
********************************************************************************/
#ifndef OPENMVO_UTILS_BOOST_JOIN_H_
#define OPENMVO_UTILS_BOOST_JOIN_H_

#ifndef BOOST_JOIN
#define BOOST_JOIN( X, Y ) BOOST_DO_JOIN( X, Y )
#define BOOST_DO_JOIN( X, Y ) BOOST_DO_JOIN2(X,Y)
#define BOOST_DO_JOIN2( X, Y ) X##Y
#endif

#endif // OPENMVO_UTILS_BOOST_JOIN_H_
