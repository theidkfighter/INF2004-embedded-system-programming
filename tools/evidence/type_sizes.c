/**
 * @file    type_sizes.c
 * @brief   Target sizeof probe for the Inter_Task_Message bodies
 *          (Requirement 23.1). Used by type_sizes.sh.
 *
 * Compiled with `arm-none-eabi-gcc -c` against the app_program/ headers of
 * either build; never linked or run. Each size is the size of one array
 * object, so `arm-none-eabi-nm -S type_sizes.o` prints the target sizeof:
 *
 *     ts_<member>__<type>    sizeof of the app_msg_body_t member <member>,
 *                            whose type is <type>
 *     ts_union__app_msg_body_t
 *     ts_tmsg__T_MSG         message header (T_MSG)
 *     ts_block__app_msg_t    Message_Pool block size: MSG_BLK_WORDS * 4 with
 *                            MSG_BLK_WORDS = (sizeof(app_msg_t) + 3) / 4 as
 *                            in os_glue.c
 *
 * The Revised_Build is recognised by the presence of msg_codec.h (design
 * C7). A _Static_assert checks that each union member really has the type
 * named in its symbol, so a renamed body type fails the compile instead of
 * printing a wrong label.
 */
#include <stdbool.h>
#include <stdint.h>

#if defined(__has_include)
#if __has_include("msg_codec.h")
#include "msg_codec.h"
#define TS_REVISED 1
#endif
#endif
#ifndef TS_REVISED
#define TS_REVISED 0
#endif

#include "app_types.h"

/* T_MSG as defined in include/tk/syscall.h; the kernel headers need the
 * whole target configuration, so the one-member layout is repeated here. */
typedef struct
{
    void *msgque[1];
} ts_t_msg_t;

/* app_msg_t as defined in os_glue.h */
typedef struct
{
    ts_t_msg_t     hdr;
    app_msg_body_t body;
} ts_app_msg_t;

#define TS_MEMBER(m)          (((app_msg_body_t *)0)->m)
#define TS_SIZE(sym, n)       char ts_##sym[(n)]
#define TS_BODY(m, type)                                                       \
    _Static_assert(__builtin_types_compatible_p(__typeof__(TS_MEMBER(m)), type), \
                   "app_msg_body_t." #m " is not " #type);                     \
    TS_SIZE(m##__##type, sizeof(TS_MEMBER(m)))

#if TS_REVISED
TS_BODY(motion, motion_msg_t);
TS_BODY(nav, nav_msg_t);
TS_BODY(obstacle, obst_msg_t);
TS_BODY(event, evt_msg_t);
#else
TS_BODY(motion, motion_cmd_t);
TS_BODY(nav, nav_event_t);
TS_BODY(obstacle, obstacle_profile_t);
TS_BODY(event, tlm_event_t);
#endif

TS_SIZE(union__app_msg_body_t, sizeof(app_msg_body_t));
TS_SIZE(tmsg__T_MSG, sizeof(ts_t_msg_t));
TS_SIZE(block__app_msg_t, ((sizeof(ts_app_msg_t) + 3u) / 4u) * 4u);
