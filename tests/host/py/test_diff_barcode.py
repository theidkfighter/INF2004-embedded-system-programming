"""Differential check of the barcode pipeline (Change Requirement 1: the
packed bc_ctx_t / bcm_t must decode exactly like the Baseline_Build).

Synthetic Code 39 passes ("*X*" for every letter, both directions, print
scales 100/75/50 %, speed jitter, ADC noise, pale bars) are fed sample by
sample to bcm_sample() in build/libis02.so and base_bcm_sample() in
build/libbase.so. Every call must return the same result, letter and
direction.
"""

import ctypes
import math
import random

import pytest

# Code 39 patterns (n = narrow, w = wide), 9 elements bar first.
C39 = {
    "*": "nwnnwnwnn",
    "A": "wnnnnwnnw", "B": "nnwnnwnnw", "C": "wnwnnwnnn", "D": "nnnnwwnnw",
    "E": "wnnnwwnnn", "F": "nnwnwwnnn", "G": "nnnnnwwnw", "H": "wnnnnwwnn",
    "I": "nnwnnwwnn", "J": "nnnnwwwnn", "K": "wnnnnnnww", "L": "nnwnnnnww",
    "M": "wnwnnnnwn", "N": "nnnnwnnww", "O": "wnnnwnnwn", "P": "nnwnwnnwn",
    "Q": "nnnnnnwww", "R": "wnnnnnwwn", "S": "nnwnnnwwn", "T": "nnnnwnwwn",
    "U": "wwnnnnnnw", "V": "nwwnnnnnw", "W": "wwwnnnnnn", "X": "nwnnwnnnw",
    "Y": "wwnnwnnnn", "Z": "nwwnwnnnn",
}

WHITE, BLACK = 200, 3900
DET_HYS, LEVELS = 0.02, 6
NARROW_MM = 2.5                          # course print, about 141 mm per "*X*"


class BcParams(ctypes.Structure):
    _fields_ = [("min_elements", ctypes.c_uint32), ("max_len_um", ctypes.c_uint32),
                ("stray_gap_um", ctypes.c_uint32), ("max_gap_um", ctypes.c_uint32)]


PAR = BcParams(9, 400000, 30000, 60000)


def bind(lib, prefix):
    P = ctypes.c_void_p
    init = getattr(lib, prefix + "bcm_init")
    init.restype, init.argtypes = None, [P, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_float, ctypes.c_uint32]
    samp = getattr(lib, prefix + "bcm_sample")
    samp.restype = ctypes.c_int
    samp.argtypes = [P, P, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_char_p, ctypes.POINTER(ctypes.c_bool)]
    return init, samp


def elements(letter, scale, rng, reverse):
    """(width_mm, is_bar) list for *letter* with quiet zones."""
    out = [(40.0, False)]
    for k, ch in enumerate("*" + letter + "*"):
        for i, p in enumerate(C39[ch]):
            w = NARROW_MM * scale * (3.0 if p == "w" else 1.0)
            out.append((w * rng.uniform(0.9, 1.1), i % 2 == 0))
        if k < 2:
            out.append((NARROW_MM * scale * rng.uniform(0.9, 1.1), False))
    out.append((80.0, False))
    if reverse:
        out.reverse()
    return out


def samples(elems, rng, pale):
    """1 kHz samples (pos_um, adc) at about 150 mm/s with jitter and noise."""
    edges, x = [], 0.0
    for w, bar in elems:
        edges.append((x, x + w, bar))
        x += w
    pos, out, i = 0.0, [], 0
    while pos < x:
        while edges[i][1] <= pos:
            i += 1
        bar = edges[i][2]
        level = BLACK if bar else WHITE
        if bar and pale and (i % 7 == 3):
            level = WHITE + int(0.35 * (BLACK - WHITE))     # a faint bar
        adc = min(4095, max(0, int(level + rng.gauss(0, 40))))
        out.append((int(pos * 1000.0), adc))
        pos += 0.15 * rng.uniform(0.6, 1.4)                  # mm per 1 ms sample
    return out


def run(init, samp, stream):
    buf = ctypes.create_string_buffer(65536)
    init(buf, WHITE, BLACK, DET_HYS, LEVELS)
    code, rev, got = ctypes.create_string_buffer(2), ctypes.c_bool(), []
    for pos, adc in stream:
        res = samp(buf, ctypes.byref(PAR), pos, adc, code, ctypes.byref(rev))
        if res != 0:
            got.append((res, code.value, rev.value, pos))
    return got


@pytest.fixture(scope="module")
def pair(lib, base):
    return bind(lib, ""), bind(base, "base_")


CASES = [(L, s, r, p) for L in sorted(C39) if L != "*" for s in (1.0, 0.75, 0.5)
         for r in (False, True) for p in (False, True)]


@pytest.mark.parametrize("letter,scale,reverse,pale", CASES)
def test_same_decode_as_baseline(pair, letter, scale, reverse, pale):
    (init, samp), (b_init, b_samp) = pair
    rng = random.Random(hash((letter, scale, reverse, pale)) & 0xFFFFFFFF)
    stream = samples(elements(letter, scale, rng, reverse), rng, pale)
    new, old = run(init, samp, stream), run(b_init, b_samp, stream)
    assert new == old
    assert math.isfinite(len(new))


def test_clean_letters_decode(pair):
    """Sanity: the synthetic stream is a real barcode (both builds read it)."""
    (init, samp), _ = pair
    rng = random.Random(7)
    hits = 0
    for letter in "AKZ":
        got = run(init, samp, samples(elements(letter, 1.0, rng, False), rng, False))
        hits += any(g[0] == 1 and g[1] == letter.encode() for g in got)
    assert hits == 3
