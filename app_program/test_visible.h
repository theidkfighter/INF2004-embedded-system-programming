/*
 * test_visible.h - TEST_VISIBLE: file-local linkage in the firmware, external
 * linkage in the host-test build (tests/host/Makefile defines HOST_TEST), so
 * the ctypes / C host tests can reach a unit or function the firmware keeps
 * private (Barr-C 1.8.a; deviation recorded in CODING_STANDARD.md).
 */
#ifndef TEST_VISIBLE_H
#define TEST_VISIBLE_H

#ifdef HOST_TEST
#define TEST_VISIBLE
#else
#define TEST_VISIBLE static
#endif

#endif /* TEST_VISIBLE_H */
