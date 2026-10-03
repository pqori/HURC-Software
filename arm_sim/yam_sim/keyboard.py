"""A procedurally built ANSI QWERTY keyboard with individually pressable keys.

Layouts: ``"full"`` (104 keys: main block, F-row, nav cluster, arrows, numpad) and ``"tkl"``
(87 keys, no numpad). Keys sit on the standard 19.05 mm (0.75 in) pitch; wide keys use the usual
ANSI widths (Backspace 2u, Tab 1.5u, Caps 1.75u, Enter 2.25u, LShift 2.25u, RShift 2.75u,
Space 6.25u, bottom-row modifiers 1.25u, numpad + / Enter 2u tall, numpad 0 2u wide).

MJCF structure (one static ``keyboard`` body, so the base never jitters)::

    <body name="keyboard" pos=... quat=...>         frame: +X along the rows (left to right as
      <geom name="keyboard_base" .../>                typed), +Y towards the F-row, +Z up; the
      <body name="key_a_body" pos=...>                origin is the centre of the base footprint
        <joint name="key_a_joint" type="slide" axis="0 0 1" range="-0.0045 0.0005"
               stiffness=... damping=.../>            spring returns the key; travel 4 mm
        <geom name="key_a" type="box" .../>          the keycap (dark sides, collides)
        <geom name="key_a_top" type="box" .../>      lighter top face (visual only)

A key counts as pressed when its joint has travelled more than ``PRESS_THRESHOLD`` (2 mm). Keys
are gravity-compensated so they rest at exactly 0 travel and their joint limits stay inactive.

Key names (``Keyboard.key_names``) are short: ``"a"``, ``"1"``, ``"space"``, ``"enter"``,
``"f1"``, ``"left"``, ``"kp_7"``, ... The MuJoCo geom is ``"key_" + name``. Every method that takes
a key name also accepts the ``key_`` prefixed form.

In the default scene the keyboard lies on the floor 0.34 m in front of the arm, rotated so that
the space bar is nearest the robot and the Esc key is on the robot's left (+Y) -- as if the arm
were sitting at the desk.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

U = 0.01905  # key pitch, m
KEY_GAP = 0.0015  # keycap footprint = n * U - gap
CAP_HALF_HEIGHT = 0.0035  # keycap box half height (7 mm tall caps)
TRAVEL = 0.004  # nominal key travel, m
PRESS_THRESHOLD = 0.002  # joint travel that counts as a press, m
BASE_HALF_HEIGHT = 0.009  # base plate is 18 mm tall
BEZEL = 0.010  # base plate border around the keys, m
KEY_MASS = 0.008  # kg
KEY_STIFFNESS = 200.0  # N/m -> 0.4 N at the 2 mm actuation point, 0.8 N at bottom-out
KEY_DAMPING = 1.2  # N s/m (about 0.5 of critical)

DEFAULT_POS = (0.34, 0.0, 0.0)
DEFAULT_YAW = -np.pi / 2  # keyboard +X (rows) -> world -Y, keyboard +Y (towards F-row) -> world +X

# Default colours: dark keycap sides, light top face, dark grey base.
CAP_RGBA = (0.10, 0.10, 0.11, 1.0)
TOP_RGBA = (0.70, 0.70, 0.68, 1.0)
BASE_RGBA = (0.16, 0.16, 0.17, 1.0)


@dataclass(frozen=True)
class KeySpec:
    name: str
    x: float  # left edge, in key units from the left of the layout
    y: float  # top edge, in key units from the top (F-row) of the layout
    w: float = 1.0
    h: float = 1.0


def _row(names: Sequence, x0: float, y: float) -> List[KeySpec]:
    out, x = [], x0
    for item in names:
        name, w = (item, 1.0) if isinstance(item, str) else item
        out.append(KeySpec(name, x, y, w))
        x += w
    return out


def ansi_layout(layout: str = "full") -> List[KeySpec]:
    """Key positions of a standard ANSI 104-key (``"full"``) or 87-key (``"tkl"``) keyboard."""
    if layout not in ("full", "tkl"):
        raise ValueError("layout must be 'full' or 'tkl'")
    keys: List[KeySpec] = []
    # Function row
    keys += [KeySpec("esc", 0, 0)]
    keys += _row([f"f{i}" for i in range(1, 5)], 2, 0)
    keys += _row([f"f{i}" for i in range(5, 9)], 6.5, 0)
    keys += _row([f"f{i}" for i in range(9, 13)], 11, 0)
    keys += _row(["prtsc", "scrlk", "pause"], 15.25, 0)
    # Main block
    y = 1.5
    keys += _row(["grave", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "minus", "equal", ("backspace", 2.0)], 0, y)
    keys += _row([("tab", 1.5), "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "lbracket", "rbracket", ("backslash", 1.5)], 0, y + 1)
    keys += _row([("caps", 1.75), "a", "s", "d", "f", "g", "h", "j", "k", "l", "semicolon", "apostrophe", ("enter", 2.25)], 0, y + 2)
    keys += _row([("lshift", 2.25), "z", "x", "c", "v", "b", "n", "m", "comma", "period", "slash", ("rshift", 2.75)], 0, y + 3)
    keys += _row([("lctrl", 1.25), ("lwin", 1.25), ("lalt", 1.25), ("space", 6.25), ("ralt", 1.25), ("rwin", 1.25), ("menu", 1.25), ("rctrl", 1.25)], 0, y + 4)
    # Navigation cluster and arrows
    keys += _row(["ins", "home", "pgup"], 15.25, y)
    keys += _row(["del", "end", "pgdn"], 15.25, y + 1)
    keys += [KeySpec("up", 16.25, y + 3)]
    keys += _row(["left", "down", "right"], 15.25, y + 4)
    if layout == "full":
        keys += _row(["numlock", "kp_divide", "kp_multiply", "kp_minus"], 18.5, y)
        keys += _row(["kp_7", "kp_8", "kp_9"], 18.5, y + 1)
        keys += [KeySpec("kp_plus", 21.5, y + 1, 1.0, 2.0)]
        keys += _row(["kp_4", "kp_5", "kp_6"], 18.5, y + 2)
        keys += _row(["kp_1", "kp_2", "kp_3"], 18.5, y + 3)
        keys += [KeySpec("kp_enter", 21.5, y + 3, 1.0, 2.0)]
        keys += [KeySpec("kp_0", 18.5, y + 4, 2.0), KeySpec("kp_decimal", 20.5, y + 4)]
    return keys


# Characters -> (key name, needs shift)
_UNSHIFTED = {" ": "space", "\n": "enter", "\t": "tab", "`": "grave", "-": "minus", "=": "equal", "[": "lbracket",
              "]": "rbracket", "\\": "backslash", ";": "semicolon", "'": "apostrophe", ",": "comma", ".": "period", "/": "slash"}
_SHIFTED = {"~": "grave", "!": "1", "@": "2", "#": "3", "$": "4", "%": "5", "^": "6", "&": "7", "*": "8", "(": "9", ")": "0",
            "_": "minus", "+": "equal", "{": "lbracket", "}": "rbracket", "|": "backslash", ":": "semicolon", '"': "apostrophe",
            "<": "comma", ">": "period", "?": "slash"}


def char_to_key(ch: str) -> Tuple[str, bool]:
    """Map a character to ``(key name, shift)``. Raises ``ValueError`` for unsupported characters."""
    if len(ch) != 1:
        raise ValueError(f"expected one character, got {ch!r}")
    if ch.isalpha() and ch.isascii():
        return ch.lower(), ch.isupper()
    if ch.isdigit():
        return ch, False
    if ch in _UNSHIFTED:
        return _UNSHIFTED[ch], False
    if ch in _SHIFTED:
        return _SHIFTED[ch], True
    raise ValueError(f"no key for character {ch!r}")


def keys_to_text(events: Iterable[str]) -> str:
    """Turn a sequence of key-press events back into text. A shift press applies to the next key."""
    inv_unshift = {v: k for k, v in _UNSHIFTED.items()}
    inv_shift = {v: k for k, v in _SHIFTED.items()}
    out, shift = [], False
    for k in events:
        k = k[4:] if k.startswith("key_") else k
        if k in ("lshift", "rshift"):
            shift = True
            continue
        if len(k) == 1 and k.isalpha():
            out.append(k.upper() if shift else k)
        elif len(k) == 1 and k.isdigit():
            out.append(inv_shift[k] if shift else k)
        elif k in inv_unshift:
            out.append(inv_shift.get(k, inv_unshift[k]) if shift and k in inv_shift else inv_unshift[k])
        else:
            out.append(f"<{k}>")
        shift = False
    return "".join(out)


def _fmt(v: Sequence[float]) -> str:
    return " ".join(f"{float(x):.9g}" for x in v)


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


class Keyboard:
    """Layout, MJCF builder and runtime helpers for the simulated keyboard.

    Build-time use (the scene builder calls this)::

        kb = Keyboard("full", pos=(0.34, 0, 0), yaw=-np.pi / 2)
        body = kb.mjcf()

    Runtime use: ``kb = Keyboard.from_model(model, data)`` (or ``kb.bind(model, data)``), then
    ``kb.pressed_keys()``, ``kb.key_pose("a")``, ``kb.key_bbox_world("space")``. Without a bound
    model the poses are the nominal (unpressed) ones computed from ``pos``/``yaw``.
    """

    BODY = "keyboard"

    def __init__(self, layout: str = "full", pos: Sequence[float] = DEFAULT_POS, yaw: float = DEFAULT_YAW,
                 cap_rgba: Sequence[float] = CAP_RGBA, top_rgba: Sequence[float] = TOP_RGBA,
                 base_rgba: Sequence[float] = BASE_RGBA, color_jitter: float = 0.0, seed: Optional[int] = 0,
                 textured: bool = False):
        self.layout = layout
        self.specs = ansi_layout(layout)
        self.key_names: List[str] = [k.name for k in self.specs]
        self._index = {n: i for i, n in enumerate(self.key_names)}
        self.pos = np.asarray(pos, dtype=float).copy()
        self.yaw = float(yaw)
        self.cap_rgba, self.top_rgba, self.base_rgba = tuple(cap_rgba), tuple(top_rgba), tuple(base_rgba)
        self.color_jitter = float(color_jitter)
        self.seed = seed
        self.textured = textured
        self.width_u = max(k.x + k.w for k in self.specs)
        self.depth_u = max(k.y + k.h for k in self.specs)
        self.size = np.array([self.width_u * U + 2 * BEZEL, self.depth_u * U + 2 * BEZEL, 2 * BASE_HALF_HEIGHT])
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None

    # ------------------------------------------------------------------ geometry (keyboard frame)
    def name(self, key: str) -> str:
        k = key[4:] if key.startswith("key_") else key
        if k not in self._index:
            raise KeyError(f"no key {key!r} on the {self.layout} layout")
        return k

    def key_local_center(self, key: str) -> np.ndarray:
        """Keycap box centre in the keyboard frame (unpressed)."""
        s = self.specs[self._index[self.name(key)]]
        x = (s.x + s.w / 2) * U - self.width_u * U / 2
        y = self.depth_u * U / 2 - (s.y + s.h / 2) * U
        z = 2 * BASE_HALF_HEIGHT + TRAVEL + CAP_HALF_HEIGHT
        return np.array([x, y, z])

    def key_half_size(self, key: str) -> np.ndarray:
        s = self.specs[self._index[self.name(key)]]
        return np.array([(s.w * U - KEY_GAP) / 2, (s.h * U - KEY_GAP) / 2, CAP_HALF_HEIGHT])

    @property
    def top_height(self) -> float:
        """Height of the keycap tops above the keyboard origin (unpressed), m."""
        return 2 * BASE_HALF_HEIGHT + TRAVEL + 2 * CAP_HALF_HEIGHT

    def frame(self) -> np.ndarray:
        """4x4 keyboard-to-world transform (from the bound model if there is one)."""
        T = np.eye(4)
        if self.model is not None and self.data is not None:
            bid = self._body_id
            T[:3, :3] = self.data.xmat[bid].reshape(3, 3)
            T[:3, 3] = self.data.xpos[bid]
            return T
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        T[:3, 3] = self.pos
        return T

    # ------------------------------------------------------------------ MJCF
    def mjcf(self) -> ET.Element:
        rng = np.random.default_rng(self.seed)
        body = ET.Element("body", {"name": self.BODY, "pos": _fmt(self.pos), "quat": _fmt(yaw_quat(self.yaw))})
        world = {"contype": "2", "conaffinity": "1", "group": "0"}
        ET.SubElement(body, "geom", {"name": "keyboard_base", "type": "box", "size": _fmt(self.size / 2),
                                     "pos": _fmt([0, 0, BASE_HALF_HEIGHT]), "rgba": _fmt(self.base_rgba), "mass": "0.8",
                                     "friction": "1 0.005 0.0001", **world})
        ET.SubElement(body, "site", {"name": "keyboard_origin", "size": "0.005", "group": "4"})
        for k in self.key_names:
            c = self.key_local_center(k)
            hs = self.key_half_size(k)
            cap, top = np.array(self.cap_rgba), np.array(self.top_rgba)
            if self.color_jitter > 0:
                j = rng.normal(0, self.color_jitter, 3)
                cap[:3] = np.clip(cap[:3] + j * 0.5, 0, 1)
                top[:3] = np.clip(top[:3] + j, 0, 1)
            kb = ET.SubElement(body, "body", {"name": f"key_{k}_body", "pos": _fmt(c), "gravcomp": "1"})
            ET.SubElement(kb, "joint", {"name": f"key_{k}_joint", "type": "slide", "axis": "0 0 1",
                                        "range": f"{-(TRAVEL + 0.0005):g} 0.0005", "stiffness": f"{KEY_STIFFNESS:g}",
                                        "springref": "0", "damping": f"{KEY_DAMPING:g}", "armature": "0.002",
                                        "solreflimit": "0.004 1", "solimplimit": "0.95 0.99 0.001"})
            ET.SubElement(kb, "geom", {"name": f"key_{k}", "type": "box", "size": _fmt(hs), "rgba": _fmt(cap),
                                       "mass": f"{KEY_MASS:g}", "friction": "0.9 0.005 0.0001", "condim": "3",
                                       "solref": "0.004 1", "solimp": "0.95 0.99 0.001", "margin": "0", **world})
            inset = min(0.0018, 0.25 * hs[0])
            top_attrs = {"name": f"key_{k}_top", "type": "box", "size": _fmt([hs[0] - inset, hs[1] - inset, 0.0004]),
                         "pos": _fmt([0, 0, CAP_HALF_HEIGHT - 0.0002]), "rgba": _fmt(top), "mass": "0",
                         "contype": "0", "conaffinity": "0", "group": "0"}
            if self.textured:
                top_attrs["material"] = "keycap_mat"
            ET.SubElement(kb, "geom", top_attrs)
        return body

    def assets(self) -> List[ET.Element]:
        """Assets the keyboard needs (a subtle noise texture for the keycap tops when ``textured``)."""
        if not self.textured:
            return []
        return [
            ET.fromstring('<texture name="keycap_tex" type="2d" builtin="flat" rgb1="1 1 1" rgb2="0.85 0.85 0.85" '
                          'mark="random" random="0.15" markrgb="0.7 0.7 0.7" width="64" height="64"/>'),
            ET.fromstring('<material name="keycap_mat" texture="keycap_tex" texrepeat="1 1" texuniform="false"/>'),
        ]

    def custom(self) -> List[ET.Element]:
        return [ET.Element("text", {"name": "keyboard_layout", "data": self.layout})]

    # ------------------------------------------------------------------ binding to a compiled model
    @classmethod
    def from_model(cls, model: mujoco.MjModel, data: Optional[mujoco.MjData] = None) -> "Keyboard":
        tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, "keyboard_layout")
        if tid < 0:
            raise ValueError("this model has no keyboard (build the scene with objects='keyboard')")
        adr, n = model.text_adr[tid], model.text_size[tid]
        layout = bytes(model.text_data[adr: adr + n - 1]).decode()
        bid = model.body(cls.BODY).id
        q = model.body_quat[bid]
        yaw = 2 * np.arctan2(q[3], q[0])
        kb = cls(layout, pos=model.body_pos[bid].copy(), yaw=yaw)
        return kb.bind(model, data)

    def bind(self, model: mujoco.MjModel, data: Optional[mujoco.MjData] = None) -> "Keyboard":
        self.model, self.data = model, data
        self._body_id = model.body(self.BODY).id
        self._qadr = np.array([model.jnt_qposadr[model.joint(f"key_{k}_joint").id] for k in self.key_names])
        self._geom = np.array([model.geom(f"key_{k}").id for k in self.key_names])
        self._top_geom = np.array([model.geom(f"key_{k}_top").id for k in self.key_names])
        self._bodies = np.array([model.body(f"key_{k}_body").id for k in self.key_names])
        return self

    def geom_to_key_index(self) -> np.ndarray:
        """Lookup array (ngeom + 1,) mapping a geom id to a key index (-1 = not a key; index -1 -> -1)."""
        lut = np.full(self.model.ngeom + 1, -1, dtype=np.int32)
        lut[self._geom] = np.arange(len(self.key_names))
        lut[self._top_geom] = np.arange(len(self.key_names))
        return lut

    def set_pose(self, pos: Sequence[float], yaw: float, model: Optional[mujoco.MjModel] = None) -> None:
        """Move the keyboard (edits ``model.body_pos/quat``; call ``mj_forward`` afterwards)."""
        self.pos, self.yaw = np.asarray(pos, dtype=float).copy(), float(yaw)
        m = model or self.model
        if m is not None:
            bid = m.body(self.BODY).id
            m.body_pos[bid] = self.pos
            m.body_quat[bid] = yaw_quat(self.yaw)

    def _data(self, data):
        d = data if data is not None else self.data
        if d is None or self.model is None:
            return None
        return d

    def travel(self, data: Optional[mujoco.MjData] = None) -> np.ndarray:
        """Downward travel of every key (m, positive = pressed), in ``key_names`` order."""
        d = self._data(data)
        if d is None:
            raise ValueError("bind the keyboard to a model and data first (Keyboard.from_model(model, data))")
        return -d.qpos[self._qadr]

    def pressed_keys(self, data: Optional[mujoco.MjData] = None, threshold: float = PRESS_THRESHOLD) -> List[str]:
        """Names of the keys pressed down by more than ``threshold`` (2 mm), in layout order."""
        t = self.travel(data)
        return [self.key_names[i] for i in np.flatnonzero(t > threshold)]

    def key_pose(self, key: str, data: Optional[mujoco.MjData] = None) -> np.ndarray:
        """4x4 world pose of the centre of the keycap's top face (keyboard orientation).

        Uses the live (possibly pressed) key position when bound to a model and data, otherwise
        the nominal unpressed pose.
        """
        k = self.name(key)
        d = self._data(data)
        T = np.eye(4)
        if d is not None:
            g = self._geom[self._index[k]]
            R = d.geom_xmat[g].reshape(3, 3)
            T[:3, :3] = R
            T[:3, 3] = d.geom_xpos[g] + R[:, 2] * CAP_HALF_HEIGHT
            return T
        F = self.frame()
        c = self.key_local_center(k) + np.array([0, 0, CAP_HALF_HEIGHT])
        T[:3, :3] = F[:3, :3]
        T[:3, 3] = F[:3, :3] @ c + F[:3, 3]
        return T

    def key_bbox_world(self, key: str, data: Optional[mujoco.MjData] = None) -> np.ndarray:
        """The 8 corners (8, 3) of the keycap box in world coordinates."""
        k = self.name(key)
        hs = self.key_half_size(k)
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
        local = signs * hs
        d = self._data(data)
        if d is not None:
            g = self._geom[self._index[k]]
            R = d.geom_xmat[g].reshape(3, 3)
            return local @ R.T + d.geom_xpos[g]
        F = self.frame()
        return (local + self.key_local_center(k)) @ F[:3, :3].T + F[:3, 3]

    def all_key_corners(self, data: Optional[mujoco.MjData] = None) -> np.ndarray:
        """Corners of every keycap, ``(n_keys, 8, 3)`` (fast path for the dataset generator)."""
        d = self._data(data)
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
        hs = np.array([self.key_half_size(k) for k in self.key_names])
        local = signs[None] * hs[:, None, :]
        if d is None:
            return np.stack([self.key_bbox_world(k) for k in self.key_names])
        R = d.geom_xmat[self._geom].reshape(-1, 3, 3)
        return np.einsum("kij,kcj->kci", R, local) + d.geom_xpos[self._geom][:, None, :]

    def set_colors(self, cap_rgba: np.ndarray, top_rgba: np.ndarray, base_rgba: Optional[Sequence[float]] = None) -> None:
        """Recolour the keycaps at runtime. ``cap_rgba``/``top_rgba``: (4,) or (n_keys, 4)."""
        m = self.model
        m.geom_rgba[self._geom] = np.broadcast_to(cap_rgba, (len(self._geom), 4))
        m.geom_rgba[self._top_geom] = np.broadcast_to(top_rgba, (len(self._top_geom), 4))
        if base_rgba is not None:
            m.geom_rgba[m.geom("keyboard_base").id] = base_rgba

    def __repr__(self) -> str:
        return f"Keyboard({self.layout!r}, {len(self.key_names)} keys, pos={self.pos.round(3).tolist()}, yaw={np.rad2deg(self.yaw):.1f} deg)"
