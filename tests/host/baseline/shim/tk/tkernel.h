/**
 * @file    tk/tkernel.h (host shim, Baseline_Build harness only)
 * @brief   The few micro T-Kernel types and calls that the Baseline_Build
 *          tsk_motion.c and os_glue.h use, so tests/host/baseline/base_motion.c
 *          can compile tsk_motion.c on the PC.
 *
 * Nothing here schedules anything: dispatch disabling is a no-op (the host
 * tests are single-threaded), tk_slp_tsk and tk_wai_flg return E_TMOUT at
 * once. The harness never calls the task body tsk_motion() or motion_wait();
 * they only have to compile and link.
 */
#ifndef BASE_SHIM_TK_TKERNEL_H
#define BASE_SHIM_TK_TKERNEL_H

#include <stddef.h>     /* NULL: the real tkernel.h provides it too */
#include <stdint.h>

typedef int32_t  INT;
typedef uint32_t UINT;
typedef int32_t  ID;
typedef int32_t  ER;
typedef int32_t  TMO;
typedef int32_t  PRI;
typedef struct t_msg { void *msgque[1]; } T_MSG;

#define LOCAL     static
#define EXPORT
#define IMPORT    extern

#define E_OK      (0)
#define E_TMOUT   (-50)
#define TMO_POL   (0)
#define TMO_FEVR  (-1)
#define TWF_ANDW  (0x00u)
#define TWF_ORW   (0x01u)

static inline ER tk_dis_dsp(void) { return E_OK; }
static inline ER tk_ena_dsp(void) { return E_OK; }
static inline ER tk_slp_tsk(TMO tmout) { (void)tmout; return E_TMOUT; }
static inline ER tk_wai_flg(ID flgid, UINT waiptn, UINT wfmode, UINT *p_flgptn, TMO tmout)
{
    (void)flgid; (void)waiptn; (void)wfmode; (void)tmout;
    *p_flgptn = 0u;
    return E_TMOUT;
}

#endif /* BASE_SHIM_TK_TKERNEL_H */
