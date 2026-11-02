"""Task 10.12: the packed barcode map g_map_w[3] (nav_map.c).

- every letter and action round-trips through the text and binary setters, in
  the design layout (letter i in word i / 10, bits 3 * (i mod 10) .. + 2);
- a parse that fails leaves the old map untouched;
- parse results, error texts and nav_map_describe() equal the Baseline_Build
  (libbase.so) for the same input;
- a reader never sees a half-updated map (crit_stub.c watches and hooks).
"""

import ctypes
import re
import string

import pytest
from hypothesis import given, strategies as st

from conftest import HOST_DIR, reset_stubs

U, I, U8, CHAR = ctypes.c_uint32, ctypes.c_int32, ctypes.c_uint8, ctypes.c_char
CP, VP = ctypes.c_char_p, ctypes.c_void_p
HOOK = ctypes.CFUNCTYPE(None)
APP_DIR = HOST_DIR.parent.parent / "app_program"

NONE, FORWARD, UTURN, LEFT, RIGHT = 0, 1, 2, 3, 4
NAMES = {FORWARD: "FWD", UTURN: "UTURN", LEFT: "LEFT", RIGHT: "RIGHT"}
SLOT_ACTIONS = (FORWARD, LEFT, RIGHT, UTURN)                # design D13 slot order
LETTERS = string.ascii_uppercase
ERR_SIZE = 64
DESCRIBE_SIZE = 256

SIGS = {
    "nav_map_clear": (None,),
    "nav_map_get": (I, CHAR),
    "nav_map_parse": (I, CP, VP, U),
    "nav_map_actions": (U, VP),
    "nav_map_set_slots": (U, VP, VP),
    "nav_map_describe": (U, VP, U),
}
LATER_FUNCTIONS = {"nav_map_actions", "nav_map_set_slots"}  # not in the Baseline_Build


def bind_all(library, prefix=""):
    for name, (res, *args) in SIGS.items():
        if prefix and name in LATER_FUNCTIONS:
            continue
        fn = getattr(library, prefix + name)
        fn.restype, fn.argtypes = res, args


class Map:
    """One nav_map build: the firmware's (libis02.so) or the Baseline_Build's."""

    def __init__(self, library, prefix=""):
        self.lib, self.prefix = library, prefix

    def call(self, name, *args):
        return getattr(self.lib, self.prefix + name)(*args)

    def clear(self):
        self.call("nav_map_clear")

    def get(self, ch):
        return self.call("nav_map_get", ch.encode())

    def parse(self, text):
        err = ctypes.create_string_buffer(b"-" * (ERR_SIZE - 1))
        rc = self.call("nav_map_parse", text.encode("latin-1"), err, ERR_SIZE)
        return rc, err.value

    def actions(self):
        act = (U8 * 26)()
        n = self.call("nav_map_actions", act)
        return n, list(act)

    def set_slots(self, present, letter):
        return self.call("nav_map_set_slots", bytes(present), bytes(letter))

    def describe(self, size=DESCRIBE_SIZE):
        buf = ctypes.create_string_buffer(b"#" * (DESCRIBE_SIZE - 1))
        n = self.call("nav_map_describe", buf, size)
        return n, buf.raw

    def state(self):
        return [self.get(c) for c in LETTERS], self.describe()


@pytest.fixture(scope="module")
def m(lib):
    bind_all(lib)
    return Map(lib)


@pytest.fixture(scope="module")
def b(base):
    bind_all(base, "base_")
    return Map(base, "base_")


def words(lib):
    return (U * 3).in_dll(lib, "g_map_w")


def model_words(actions):
    """g_map_w for a list of 26 actions, from the design layout."""
    out = [0, 0, 0]
    for i, act in enumerate(actions):
        out[i // 10] |= act << (3 * (i % 10))
    return out


def pairs_text(mapping):
    return "MAP:" + ",".join(f"{letter}={NAMES[act]}" for letter, act in mapping.items())


# ---- round trips ------------------------------------------------------------

@pytest.mark.parametrize("act", [FORWARD, UTURN, LEFT, RIGHT], ids=NAMES.get)
@pytest.mark.parametrize("letter", LETTERS)
def test_every_letter_and_action_round_trips(m, lib, letter, act):
    want = [act if c == letter else NONE for c in LETTERS]
    assert m.parse(f"{letter}={NAMES[act]}")[0] == 1
    assert m.get(letter) == act
    assert m.get(letter.lower()) == act
    assert m.actions() == (1, want)
    assert list(words(lib)) == model_words(want)
    m.clear()
    assert m.get(letter) == NONE
    assert list(words(lib)) == [0, 0, 0]


@pytest.mark.parametrize("slot", range(4), ids=[NAMES[a] for a in SLOT_ACTIONS])
@pytest.mark.parametrize("code", range(26), ids=list(LETTERS))
def test_every_letter_round_trips_through_the_binary_slots(m, lib, slot, code):
    present, letter = [0] * 4, [0] * 4
    present[slot], letter[slot] = 1, code
    want = [SLOT_ACTIONS[slot] if i == code else NONE for i in range(26)]
    assert m.set_slots(present, letter) == 1
    assert m.actions() == (1, want)
    assert list(words(lib)) == model_words(want)


def test_all_26_letters_at_once_use_30_bits_of_each_word(m, lib):
    actions = [RIGHT] * 26
    assert m.parse(pairs_text({c: RIGHT for c in LETTERS}))[0] == 26
    assert m.actions() == (26, actions)
    w = list(words(lib))
    assert w == model_words(actions)
    assert all(x >> 30 == 0 for x in w[:2]) and w[2] >> 18 == 0


def test_a_letter_given_twice_keeps_its_last_action(m):
    """The bits of the first action must not stay behind (LEFT = 3, RIGHT = 4)."""
    assert m.parse("A=LEFT,A=RIGHT")[0] == 1
    assert m.get("A") == RIGHT


@given(st.dictionaries(st.sampled_from(LETTERS), st.sampled_from(sorted(NAMES)), min_size=1))
def test_random_maps_round_trip(m, lib, mapping):
    want = [mapping.get(c, NONE) for c in LETTERS]
    assert m.parse(pairs_text(mapping))[0] == len(mapping)
    assert m.actions() == (len(mapping), want)
    assert [m.get(c) for c in LETTERS] == want
    assert list(words(lib)) == model_words(want)


@pytest.mark.parametrize("ch", ["@", "[", "`", "{", "0", " ", "\x00"])
def test_a_character_outside_a_to_z_has_no_action(m, ch):
    m.parse("A=FWD,Z=LEFT")
    assert m.get(ch) == NONE


# ---- a failed parse changes nothing -----------------------------------------

BAD_TEXTS = [
    "",                                        # empty map
    "MAP:",
    "   ",
    "A=FWD,1=LEFT",                            # letters must be A-Z
    "A=FWD,B=SIDEWAYS",                        # unknown movement
    "A=FWD,B=",                                # "expected L=ACTION or a single letter"
    "A=FWD,BC=LEFT",
    "A=FWD,B",                                 # the two forms mixed
    "F,U,K",                                   # three positional letters
    "F,U,K,R,S",                               # five
    "F,U,K,F",                                 # the same letter twice
]


@pytest.mark.parametrize("text", BAD_TEXTS)
def test_failed_parse_leaves_the_old_map_untouched(m, lib, text):
    old = {"B": FORWARD, "K": LEFT, "U": RIGHT, "Z": UTURN}
    assert m.parse(pairs_text(old))[0] == 4
    before_state, before_words = m.state(), list(words(lib))
    reset_stubs(lib)
    rc, err = m.parse(text)
    assert rc == -1 and err != b""
    assert lib.crit_stub_dsp_enters() == 0                  # nothing stored at all
    assert m.state() == before_state
    assert m.actions()[0] == 4
    assert list(words(lib)) == before_words


# ---- same results as the Baseline_Build -------------------------------------

ITEMS = st.one_of(
    st.builds(lambda c, a: f"{c}={a}", st.sampled_from(LETTERS + LETTERS.lower() + "1?"),
              st.sampled_from(["FWD", "forward", "STRAIGHT", "f", "LEFT", "l", "RIGHT", "r", "UTURN",
                               "U-TURN", "u_turn", "U", "BACK", "NOPE", ""])),
    st.sampled_from(list(LETTERS)),
    st.sampled_from(["CLEAR", "clear", "MAP:", "AB", "=", "A=", "é"]),
)
TEXTS = st.builds(lambda pre, items, sep: pre + sep.join(items),
                  st.sampled_from(["", "MAP:", " ", "MAP: "]),
                  st.lists(ITEMS, max_size=30),
                  st.sampled_from([",", ";", " ", "\t", ", ", "\r\n"]))


# Truncated output: the separator, letter and name checks all have their own limit.
SIZES = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 20, 41, 100, DESCRIBE_SIZE]


@given(st.lists(TEXTS, min_size=1, max_size=4))
def test_parse_and_describe_equal_the_baseline(m, b, texts):
    m.clear()
    b.clear()
    for text in texts:
        assert m.parse(text) == b.parse(text), text
        assert m.state() == b.state(), text
        got = [m.get(c) for c in LETTERS]
        assert m.actions() == (sum(a != NONE for a in got), got), text
    for size in SIZES:
        assert m.describe(size) == b.describe(size), size


@given(st.lists(st.integers(0, 1), min_size=4, max_size=4),
       st.lists(st.integers(0, 40), min_size=4, max_size=4))
def test_binary_slots_equal_the_same_map_as_baseline_text(m, b, present, letter):
    """nav_map_set_slots is not in the Baseline_Build (it had text commands
    only), so the same map goes through the Baseline_Build text parser. A letter
    above Z is ignored; a repeated letter keeps the last slot, as a repeated
    pair does in the text form."""
    n = m.set_slots(present, letter)
    items = [f"{chr(65 + code)}={NAMES[SLOT_ACTIONS[k]]}"
             for k, code in enumerate(letter) if present[k] and code < 26]
    if items:
        assert b.parse(",".join(items))[0] == n
    else:
        b.clear()
        assert n == 0
    assert m.state() == b.state()


def test_describe_golden_text(m, b):
    for lib_map in (m, b):
        assert lib_map.parse("MAP:F,U,K,R")[0] == 4
        n, raw = lib_map.describe()
        assert (n, raw.split(b"\0")[0]) == (4, b"F=FWD,K=LEFT,R=RIGHT,U=UTURN")


def test_describe_of_an_empty_map_is_an_empty_string(m):
    m.clear()
    n, raw = m.describe()
    assert (n, raw[:1]) == (0, b"\0")


def test_describe_with_size_zero_writes_nothing(m):
    m.parse("A=FWD")
    assert m.describe(0) == (0, b"#" * (DESCRIBE_SIZE - 1) + b"\0")


# ---- a reader never sees a half-updated map ---------------------------------

UPDATES = {
    "parse": lambda m: m.parse("A=FWD,K=LEFT,U=RIGHT"),
    "parse_clear": lambda m: m.parse("MAP:CLEAR"),
    "clear": lambda m: m.clear(),
    "slots": lambda m: m.set_slots([1, 1, 1, 0], [0, 10, 20, 0]),
}


@pytest.mark.parametrize("name", UPDATES)
def test_all_three_words_change_in_one_section(m, lib, name):
    m.parse(pairs_text({c: UTURN for c in LETTERS}))
    reset_stubs(lib)
    assert lib.crit_stub_watch(ctypes.addressof(words(lib)), 12)
    UPDATES[name](m)
    assert lib.crit_stub_dsp_enters() == 1
    assert lib.crit_stub_changed_sections() == 1
    assert lib.crit_stub_outside_writes() == 0
    assert lib.crit_stub_errors() == 0
    assert lib.crit_stub_dsp_depth() == 0


@pytest.mark.parametrize("read", [
    lambda m: m.actions()[1],
    lambda m: [ord(x) for x in m.describe()[1].split(b"\0")[0].decode()],
], ids=["nav_map_actions", "nav_map_describe"])
def test_whole_map_readers_copy_inside_one_section(m, lib, read):
    """The enter hook stands for a writer that finished just before the section,
    the exit hook for one that runs just after it: only the first may be seen."""
    m.parse(pairs_text({c: FORWARD for c in LETTERS}))
    old = list(words(lib))
    m.parse(pairs_text({c: RIGHT for c in LETTERS}))
    new = list(words(lib))
    want = read(m)                                          # the new map, no hooks
    m.clear()
    reset_stubs(lib)
    w = words(lib)

    def before():
        w[:] = new

    def after():
        w[:] = old

    enter, leave = HOOK(before), HOOK(after)
    lib.crit_stub_set_hooks(enter, leave)
    got = read(m)
    reset_stubs(lib)
    assert got == want


def test_a_single_letter_read_is_one_load_with_no_section(m, lib):
    m.parse("A=FWD")
    reset_stubs(lib)
    assert m.get("A") == FORWARD
    assert lib.crit_stub_dsp_enters() == 0


def test_sections_are_loads_and_stores_only():
    """Interrupt and dispatch rule: no loop or division inside a section."""
    src = re.sub(r"/\*.*?\*/", " ", (APP_DIR / "nav_map.c").read_text(encoding="utf-8"), flags=re.S)
    sections = re.findall(r"crit_dsp_enter\(\);(.*?)crit_dsp_exit\(\);", src, flags=re.S)
    assert len(sections) == 2
    for body in sections:
        assert not re.search(r"\b(for|while|do)\b|[/%]", body)


def test_old_byte_map_is_gone():
    src = (APP_DIR / "nav_map.c").read_text(encoding="utf-8")
    assert not re.search(r"\bg_map\s*\[", src)


@pytest.fixture(autouse=True)
def stubs_clean(lib):
    reset_stubs(lib)
    yield
    reset_stubs(lib)
