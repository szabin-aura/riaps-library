//
// Created by riaps on 11/14/17.
//
#ifdef __cplusplus
extern "C"
{
#endif
#ifndef TIMETEST_TIME_MACROS_H_H
#define TIMETEST_TIME_MACROS_H_H
#include <time.h>
#include <errno.h>
typedef struct timespec system_timespec;
typedef __int64_t time_ns_t;
#include <limits.h>

#define ZERO_TIME {0,0}

/** These are platform specific limits and can be modified based on the underlying time representation that can be used on that platform */
#define TIME_T_MAX     (LONG_MAX)
#define INFINITE_TIME {TIME_T_MAX,0}
#define MAX_TIME_NS_T_SECONDS 9223372036
#define is_zero(ts) \
    (t.tv_sec==0 && t.tv_nsec==0)
#define is_INFINITE(ts) \
    (t.tv_sec>=LONG_MAX)

#define MSEC_PER_SEC    1000L
#define USEC_PER_MSEC    1000L
#define NSEC_PER_USEC    1000L
#define NSEC_PER_MSEC    1000000L
#define USEC_PER_SEC    1000000L
#define NSEC_PER_SEC    1000000000L
#define FSEC_PER_SEC    1000000000000000LL

#define system_time_valid(ts) \
    (((ts)->tv_sec >= 0) && ((ts)->tv_sec<TIME_T_MAX) && (((unsigned long) (ts)->tv_nsec) < NSEC_PER_SEC))

/**
* @brief compares two  timestamps
* @param [in] t1 pointer to the LHS value
* @param [in] t2 pointer to the RHS value
* @retval EINVAL Either t1 or t2 is NULL
* @retval 1 *t1 > *t2
* @retval -1 *t1 < *t2
* @retval 0 *t1 == *t2
*/
inline int ts_compare(const system_timespec *t1,
                      const system_timespec *t2) {
    if (t1 == 0 || t2 == 0)
        return (EINVAL);
    if (t1->tv_sec < t2->tv_sec)
        return (-1);        /* Less than. */
    else if (t1->tv_sec > t2->tv_sec)
        return (1);        /* Greater than. */
    else if (t1->tv_nsec < t2->tv_nsec)
        return (-1);        /* Less than. */
    else if (t1->tv_nsec > t2->tv_nsec)
        return (1);        /* Greater than. */
    else
        return (0);        /* Equal. */
}


/**
 *@brief  set system_timespec sec and nsec parts and normalize
 *
 *@param [out] ts         pointer to timespec variable to be set
 *@param [in] sec        seconds to set
 *@param [in] nsec       nanoseconds to set
 *
 *@details  Set seconds and nanoseconds field of a timespec variable and
 * normalize to the timespec storage format
 *
 * Note: The tv_nsec part is always in the range of
 *      0 <= tv_nsec < NSEC_PER_SEC
 * For negative values only the tv_sec field is negative !
 * @retval EINVAL Either ts is NULL
 * @retval EOVERFLOW sec and nsec are too large and cannot be returned as system_timespec 
 */
inline int set_normalized_system_timespec(system_timespec *ts,
                                          time_t sec, long nsec) {
    if (!ts)
        return EINVAL;

    while (nsec >= NSEC_PER_SEC) {
        nsec -= NSEC_PER_SEC;
        if (sec >= TIME_T_MAX)
            return EOVERFLOW;
        ++sec;
    }
    while (nsec < 0) {
        nsec += NSEC_PER_SEC;
        --sec;
    }
    ts->tv_sec = sec;
    ts->tv_nsec = nsec;
    return 0;
}
/**
* @brief convert a positive timestamp to a 64 bit nanoseconds number
* @param [in] t1 pointer to the system time value
* @param [out] ns pointer to the converted nanosecond value
* @retval EINVAL Either t1 or ns is NULL
* @retval 0 conversion successful
* @retval EOVERFLOW ts is too large and cannot be returned as ns 
* @retval EINVAL ts is not a valid system time
*/
inline int system_timespec_to_time_ns(const system_timespec *t1,
                                      __int64_t *ns) {
    if (!t1 || !ns)
        return EINVAL;

    if (!system_time_valid (t1))
        return EINVAL;

    *ns = ((__int64_t) t1->tv_sec * NSEC_PER_SEC) + t1->tv_nsec;
    return 0;
}

/**
* @brief convert a positive  64 bit nanoseconds number to F6 system time
* @param [in] ns pointer to the  nanosecond value
* @param [out] t1 pointer to the converted F6 system time value
* @retval EINVAL Either t1 or ns is NULL
* @retval 0 conversion successful
* @retval EOVERFLOW ts is too large and cannot be returned as system_timespec 
* @retval EINVAL ts is not a valid system time
*/
inline int time_ns_to_system_time(const time_ns_t *ns,
                                  system_timespec *t1) {
    if (!t1 || !ns)
        return EINVAL;

    __int64_t seconds = (*ns) / NSEC_PER_SEC;
    long rem = (*ns) % NSEC_PER_SEC;
    if (seconds > TIME_T_MAX
        || (seconds == TIME_T_MAX && rem >= NSEC_PER_SEC))
        return EOVERFLOW;

    set_normalized_system_timespec(t1, seconds, rem);

    return 0;
}
/**
* @brief adds two system_timespec: Z=X+Y
* @param [in] X pointer to one of the time stamps
* @param [in] Y pointer to the other time stamp
* @param [out] Z pointer to the the result
* @retval EINVAL Either X or Y or Z is NULL
* @retval 0 conversion successful
* @retval EOVERFLOW ts is too large and cannot be returned as system_timespec 
* @retval EINVAL ts is not a valid system time
*/
inline int system_time_add(const system_timespec *X,
                           const system_timespec *Y,
                           system_timespec *Z) {
    if (!X || !Y || !Z)
        return EINVAL;

    if (!system_time_valid (X) || !system_time_valid (Y))
        return EINVAL;


    __int64_t temp = (__int64_t) X->tv_sec + Y->tv_sec;
    long nsecs = X->tv_nsec + Y->tv_nsec;

    if ((temp > TIME_T_MAX)
        || ((temp == TIME_T_MAX) && (nsecs >= NSEC_PER_SEC)))
        return EOVERFLOW;

    set_normalized_system_timespec(Z, temp, nsecs);
    return 0;
}

/**
* @brief adds two system_timespec: Z=X-Y
* @param [in] X pointer to one of the time stamps
* @param [in] Y pointer to the other time stamp
* @param [out] Z pointer to the the result
* @retval EINVAL Either X or Y or Z is NULL
* @retval 0 conversion successful
* @retval EOVERFLOW ts is too large and cannot be returned as system_timespec 
* @retval EINVAL ts is not a valid system time
*/
inline int system_time_sub(const system_timespec *X,
                           const system_timespec *Y,
                           system_timespec *Z) {
    if (!X || !Y || !Z)
        return EINVAL;

    if (!system_time_valid (X) || !system_time_valid (Y))
        return EINVAL;

    __int64_t temp = (__int64_t) X->tv_sec - Y->tv_sec;
    long nsecs = X->tv_nsec - Y->tv_nsec;

    set_normalized_system_timespec(Z, temp, nsecs);
    return 0;
}
/*
   * @}
    */
#ifdef __cplusplus
}                // extern C
#endif


#endif //TIMETEST_TIME_MACROS_H_H
