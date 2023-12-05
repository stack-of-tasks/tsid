#ifndef __TSID_MACROS_HPP__
#define __TSID_MACROS_HPP__

// ref https://www.fluentcpp.com/2019/08/30/how-to-disable-a-warning-in-cpp/
#if defined(_MSC_VER)

#define TSID_DISABLE_WARNING_PUSH __pragma(warning(push))
#define TSID_DISABLE_WARNING_POP __pragma(warning(pop))
#define TSID_DISABLE_WARNING(warningNumber) \
  __pragma(warning(disable : warningNumber))
#define TSID_DISABLE_WARNING_DEPRECATED TSID_DISABLE_WARNING(4996)

#elif defined(__GNUC__) || defined(__clang__)

#define TSID_DO_PRAGMA(X) _Pragma(#X)
#define TSID_DISABLE_WARNING_PUSH TSID_DO_PRAGMA(GCC diagnostic push)
#define TSID_DISABLE_WARNING_POP TSID_DO_PRAGMA(GCC diagnostic pop)
#define TSID_DISABLE_WARNING(warningName) \
  TSID_DO_PRAGMA(GCC diagnostic ignored #warningName)
// clang-format off
#define TSID_DISABLE_WARNING_DEPRECATED \
  TSID_DISABLE_WARNING(-Wdeprecated-declarations)
// clang-format on

#else

#define TSID_DISABLE_WARNING_PUSH
#define TSID_DISABLE_WARNING_POP
#define TSID_DISABLE_WARNING_DEPRECATED

#endif

#endif
