#!/usr/bin/env python3
"""
SDF World Editor - open a Gazebo world (.world / .sdf), see it in 3D, edit it,
and every change is written straight back into the SDF file.

    python3 sdf_world_editor.py path/to/my.world
    python3 sdf_world_editor.py my.world --port 8765 --model-path ~/my_models --no-browser
    python3 sdf_world_editor.py new_world.world      # creates a blank world if it doesn't exist

Needs only the Python standard library (Python 3.9+). The 3D view opens in your web
browser (three.js is loaded from a CDN, so the browser needs internet access once).

In the browser:
  click a model ......... select it              W / E .......... move / rotate gizmo
  drag the gizmo ........ move / rotate it       F .............. focus selected
  right panel ........... exact pose, size,      Del ............ delete selected
                          colour, raw XML        Ctrl+D ......... duplicate
  Live save ON .......... every edit is saved    Ctrl+Z / Ctrl+Y  undo / redo
                          to the file instantly  Ctrl+S ......... save (when Live save is off)

A copy of the original file is kept as <file>.bak the first time it is opened.
If the file changes on disk (e.g. you edit it in VS Code) the view reloads by itself.
"""
import argparse
import copy
import glob
import json
import math
import mimetypes
import os
import re
import shutil
import sys
import tempfile
import threading
import webbrowser
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, quote, unquote, urlparse


# --------------------------------------------------------------------------- XML helpers
def parse_fragment(text):
    """Parse XML text, keeping comments so they survive a save."""
    p = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    p.feed(text)
    return p.close()


def kids(el):
    return [c for c in el if isinstance(c.tag, str)]


def ftext(el, path, default=None):
    if el is None:
        return default
    t = el.findtext(path)
    return t.strip() if t is not None and t.strip() else default


def floats(s, n=None, default=None):
    try:
        v = [float(x) for x in (s or "").split()]
    except ValueError:
        return default
    if n and len(v) < n:
        return default
    return v[:n] if n else v


def num(s, default):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def fmt(v):
    s = f"{float(v):.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def fmt_list(vs):
    return " ".join(fmt(v) for v in vs)


def _ws(s):
    return s if (s is not None and s.strip() == "") else None


def _layout(parent, depth):
    """Indentation used inside `parent`: (child indent, closing indent, unit)."""
    ks = list(parent)
    inner = _ws(parent.text) if ks else None
    closing = _ws(ks[-1].tail) if ks else None
    if inner is not None and "\n" not in inner:
        inner = None
    if closing is not None and "\n" not in closing:
        closing = None
    if inner and closing and inner.startswith(closing) and len(inner) > len(closing):
        unit = inner[len(closing):]
    else:
        unit = "  "
    if closing is None:
        closing = "\n" + unit * depth
    if inner is None:
        inner = closing + unit
    return inner, closing, unit


def _reindent(el, inner, unit):
    for e in el.iter():
        if len(e) and _ws(e.text) is not None:
            e.text = None
        if e is not el and _ws(e.tail) is not None:
            e.tail = None
    base = inner.lstrip("\n")
    level = 0
    if unit and base and base == unit * (len(base) // len(unit)):
        level = len(base) // len(unit)
    ET.indent(el, space=unit, level=level)


def insert_pretty(parent, child, index=None, depth=1):
    inner, closing, unit = _layout(parent, depth)
    _reindent(child, inner, unit)
    ks = list(parent)
    if index is None or index >= len(ks):
        if ks:
            ks[-1].tail = inner
        else:
            parent.text = inner
        child.tail = closing
        parent.append(child)
    else:
        child.tail = inner
        parent.insert(index, child)


def remove_pretty(parent, child):
    ks = list(parent)
    i = ks.index(child)
    if i == len(ks) - 1:
        if i > 0:
            ks[i - 1].tail = child.tail
        elif _ws(parent.text) is not None:
            parent.text = None
    parent.remove(child)


# --------------------------------------------------------------------------- pose helpers
def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sp) if abs(sp) >= 1 else math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def parse_pose(pe):
    if pe is None or not (pe.text or "").strip():
        return [0.0] * 6
    v = floats(pe.text) or []
    if pe.get("rotation_format", "") == "quat_xyzw" and len(v) >= 7:
        return v[:3] + list(quat_to_rpy(*v[3:7]))
    v = (v + [0.0] * 6)[:6]
    if pe.get("degrees", "").lower() in ("true", "1"):
        v[3:] = [math.radians(a) for a in v[3:]]
    return v


def write_pose(pe, pose):
    pose = [float(x) for x in pose]
    if pe.get("rotation_format") == "quat_xyzw":
        del pe.attrib["rotation_format"]
    if pe.get("degrees", "").lower() in ("true", "1"):
        pose = pose[:3] + [math.degrees(a) for a in pose[3:]]
    pe.text = fmt_list(pose)


# --------------------------------------------------------------------------- colours
SCRIPT_COLORS = {
    "white": [1, 1, 1], "black": [0.05, 0.05, 0.05], "darkgrey": [0.35, 0.35, 0.35],
    "grey": [0.7, 0.7, 0.7], "gray": [0.7, 0.7, 0.7], "red": [0.85, 0.1, 0.1],
    "green": [0.1, 0.7, 0.1], "blue": [0.1, 0.2, 0.85], "yellow": [0.95, 0.85, 0.1],
    "orange": [1, 0.5, 0.05], "purple": [0.5, 0.1, 0.6], "turquoise": [0.1, 0.75, 0.7],
    "gold": [0.85, 0.65, 0.15], "wood": [0.55, 0.4, 0.25], "brick": [0.6, 0.25, 0.2],
    "road": [0.25, 0.25, 0.25], "asphalt": [0.25, 0.25, 0.25], "indigo": [0.3, 0.1, 0.55],
}


def visual_color(vis):
    m = vis.find("material")
    if m is None:
        return None
    for tag in ("diffuse", "ambient"):
        c = floats(ftext(m, tag, ""), 3)
        if c:
            full = floats(ftext(m, tag, "")) or c
            return (full + [1.0])[:4]
    name = (ftext(m, "script/name") or "").lower()
    if name:
        key = name.split("/")[-1].replace("transparent", "")
        for word in sorted(SCRIPT_COLORS, key=len, reverse=True):
            if word in key:
                return SCRIPT_COLORS[word] + [0.5 if "transparent" in name else 1.0]
    return None


# --------------------------------------------------------------------------- resolving model:// etc
class Resolver:
    def __init__(self, world_path, extra_paths):
        self.world_dir = os.path.dirname(os.path.abspath(world_path))
        wd = self.world_dir
        paths = [os.path.expanduser(p) for p in extra_paths]
        for var in ("GZ_SIM_RESOURCE_PATH", "IGN_GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_PATH",
                    "SDF_PATH", "GZ_FILE_PATH"):
            paths += [p for p in os.environ.get(var, "").split(os.pathsep) if p]
        paths += [wd, os.path.join(wd, "models"), os.path.join(wd, "..", "models"),
                  os.path.join(wd, "..", "..", "models"), os.path.expanduser("~/.gazebo/models")]
        paths += sorted(glob.glob("/usr/share/gazebo*/models"))
        seen, self.model_paths = set(), []
        for p in paths:
            ap = os.path.abspath(p)
            if os.path.isdir(ap) and ap not in seen:
                seen.add(ap)
                self.model_paths.append(ap)
        self.allowed = {os.path.realpath(wd)}
        self.cache, self.pkg_cache = {}, {}

    # ---- packages (package://pkg/...)
    def find_package(self, pkg):
        if pkg in self.pkg_cache:
            return self.pkg_cache[pkg]
        found = None
        for pre in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            d = os.path.join(pre, "share", pkg)
            if pre and os.path.isdir(d):
                found = d
                break
        if not found:
            for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
                d = os.path.join(root, pkg)
                if root and os.path.isdir(d):
                    found = d
                    break
        if not found:  # walk up from the world file, then look through a workspace src/
            d = self.world_dir
            for _ in range(8):
                if self._pkg_name(os.path.join(d, "package.xml")) == pkg:
                    found = d
                    break
                src = os.path.join(d, "src")
                if os.path.isdir(src):
                    found = self._scan_src(src, pkg)
                    if found:
                        break
                nd = os.path.dirname(d)
                if nd == d:
                    break
                d = nd
        if found:
            self.allowed.add(os.path.realpath(found))
        self.pkg_cache[pkg] = found
        return found

    @staticmethod
    def _pkg_name(px):
        if not os.path.isfile(px):
            return None
        try:
            m = re.search(r"<name>\s*([^<\s]+)\s*</name>", open(px, encoding="utf-8").read())
            return m.group(1) if m else None
        except OSError:
            return None

    def _scan_src(self, src, pkg):
        base_depth = src.rstrip(os.sep).count(os.sep)
        for root, dirs, files in os.walk(src):
            if root.count(os.sep) - base_depth >= 4:
                dirs[:] = []
            dirs[:] = [d for d in dirs if not d.startswith(".") and d not in ("build", "install", "log")]
            if "package.xml" in files and self._pkg_name(os.path.join(root, "package.xml")) == pkg:
                return root
        return None

    # ---- Fuel cache
    def fuel_dir(self, uri):
        m = re.match(r"https?://([^/]+)/(?:[\d.]+/)?([^/]+)/models/([^/]+)(?:/(\d+|tip))?(?:/files/(.*))?",
                     unquote(uri))
        if not m:
            return None, ""
        host, owner, name, ver, rest = m.groups()
        for base in ("~/.gz/fuel", "~/.ignition/fuel"):
            for o, n in ((owner.lower(), name.lower()), (owner, name)):
                d = os.path.join(os.path.expanduser(base), host, o, "models", n)
                if os.path.isdir(d):
                    vers = [v for v in os.listdir(d) if v.isdigit()]
                    if ver and ver.isdigit() and ver in vers:
                        return os.path.join(d, ver), rest or ""
                    if vers:
                        return os.path.join(d, max(vers, key=int)), rest or ""
        return None, ""

    def model_dir(self, uri):
        if uri.startswith("model://"):
            name, _, rest = uri[8:].partition("/")
            for p in self.model_paths:
                d = os.path.join(p, name)
                if os.path.isdir(d):
                    return d, rest
            return None, ""
        if uri.startswith(("http://", "https://")):
            return self.fuel_dir(uri)
        path = uri[7:] if uri.startswith("file://") else uri
        if not os.path.isabs(path):
            path = os.path.join(self.world_dir, path)
        if os.path.isdir(path):
            return path, ""
        if os.path.isfile(path):
            return os.path.dirname(path), os.path.basename(path)
        return None, ""

    def model_sdf_file(self, d):
        cfg = os.path.join(d, "model.config")
        if os.path.isfile(cfg):
            try:
                root = parse_fragment(open(cfg, encoding="utf-8").read())
                sdfs = root.findall("sdf")

                def ver(e):
                    return tuple(int(x) for x in re.findall(r"\d+", e.get("version", "0")))
                for e in sorted(sdfs, key=ver, reverse=True):
                    f = os.path.join(d, (e.text or "").strip())
                    if os.path.isfile(f):
                        return f
            except ET.ParseError:
                pass
        for f in [os.path.join(d, "model.sdf")] + sorted(glob.glob(os.path.join(d, "*.sdf"))):
            if os.path.isfile(f):
                return f
        return None

    def load_model(self, uri):
        """-> (model_dir, root element of its SDF) or (None, None)."""
        if uri in self.cache:
            return self.cache[uri]
        result = (None, None)
        d, rest = self.model_dir(uri)
        if d:
            f = os.path.join(d, rest) if rest and os.path.isfile(os.path.join(d, rest)) else self.model_sdf_file(d)
            if f:
                try:
                    root = parse_fragment(open(f, encoding="utf-8").read())
                    self.allowed.add(os.path.realpath(d))
                    result = (d, root)
                except (OSError, ET.ParseError) as e:
                    print(f"  ! could not parse {f}: {e}")
        self.cache[uri] = result
        return result

    def resolve_file(self, uri, base_dir):
        p = None
        if uri.startswith("model://") or uri.startswith(("http://", "https://")):
            d, rest = self.model_dir(uri)
            if d:
                self.allowed.add(os.path.realpath(d))
                p = os.path.join(d, rest)
        elif uri.startswith("package://"):
            pkg, _, rest = uri[10:].partition("/")
            d = self.find_package(pkg)
            p = os.path.join(d, rest) if d else None
        elif uri.startswith("file://"):
            p = uri[7:]
        else:
            p = uri if os.path.isabs(uri) else os.path.join(base_dir, uri)
        if p and os.path.isfile(p):
            p = os.path.realpath(p)
            self.allowed.add(os.path.dirname(p))
            self.allowed.add(os.path.dirname(os.path.dirname(p)))  # ../materials/textures
            return p
        return None

    def is_allowed(self, p):
        rp = os.path.realpath(p)
        for a in self.allowed:
            if rp == a or rp.startswith(a.rstrip(os.sep) + os.sep):
                return True
        return False


def file_url(p):
    pp = p.replace("\\", "/")
    if not pp.startswith("/"):
        pp = "/" + pp
    return "/file" + quote(pp)


# --------------------------------------------------------------------------- the document
BLANK_WORLD = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
"""

INERTIA = {"box": (0.166667, 0.166667, 0.166667), "cylinder": (0.145833, 0.145833, 0.125),
           "sphere": (0.1, 0.1, 0.1)}
GEOM_XML = {"box": "<box><size>1 1 1</size></box>",
            "cylinder": "<cylinder><radius>0.5</radius><length>1</length></cylinder>",
            "sphere": "<sphere><radius>0.5</radius></sphere>"}
NEW_COLOR = {"box": "0.8 0.3 0.1 1", "cylinder": "0.1 0.45 0.8 1", "sphere": "0.2 0.7 0.3 1"}
GEOM_KEYS = {"box": {"size": 3}, "cylinder": {"radius": 1, "length": 1}, "sphere": {"radius": 1},
             "capsule": {"radius": 1, "length": 1}, "ellipsoid": {"radii": 3},
             "plane": {"size": 2, "normal": 3}, "mesh": {"scale": 3}}


class WorldDoc:
    def __init__(self, path, resolver):
        self.path = os.path.realpath(path)  # follow colcon --symlink-install links to the real source file
        self.res = resolver
        self.lock = threading.RLock()
        self.undo, self.redo = [], []
        self.version, self.autosave, self.dirty = 0, True, False
        self.notice, self.reg, self._n, self._warn = None, {}, 0, []
        self.load_from_disk()
        bak = self.path + ".bak"
        if not os.path.exists(bak):
            shutil.copy2(self.path, bak)

    # ---- load / save
    def load_from_disk(self):
        with open(self.path, encoding="utf-8") as f:
            raw = f.read()
        self._load_text(raw)
        self.mtime = os.path.getmtime(self.path)
        self.dirty = False

    def _load_text(self, raw):
        for prefix, uri in re.findall(r'xmlns:([\w.-]+)\s*=\s*"([^"]+)"', raw):
            try:
                ET.register_namespace(prefix, uri)
            except ValueError:
                pass
        m = re.search(r"<(?![?!])", raw)
        self.prolog = raw[:m.start()] if m else ""
        self.root = parse_fragment(raw)

    def serialize(self):
        return self.prolog + ET.tostring(self.root, encoding="unicode") + "\n"

    def save(self):
        text = self.serialize()
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path), prefix=".sdfedit_", suffix=".tmp")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
        try:
            shutil.copymode(self.path, tmp)
        except OSError:
            pass
        os.replace(tmp, self.path)
        self.mtime = os.path.getmtime(self.path)
        self.dirty = False

    def check_external(self):
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            return
        if m == self.mtime:
            return
        if self.dirty:
            self.mtime = m
            self.notice = "File changed on disk but you have unsaved edits - kept yours. Save to overwrite, or Reload."
            self.version += 1
            return
        try:
            before = self.serialize()
            self.load_from_disk()
            self.undo.append(before)
            self.notice = "File changed on disk - reloaded"
        except (OSError, ET.ParseError) as e:
            self.mtime = m
            self.notice = f"File changed on disk but could not be parsed ({e}) - kept the last good version"
        self.version += 1

    @property
    def container(self):
        r = self.root
        if r.tag == "world":
            return r
        w = r.find("world")
        return w if w is not None else r

    def parent_of(self, el):
        for p in self.root.iter():
            for c in p:
                if c is el:
                    return p
        return None

    def depth_of(self, el):
        d, cur = 0, el
        while cur is not None and cur is not self.root:
            cur = self.parent_of(cur)
            d += 1
        return d

    # ---- scene description for the browser
    def _reg(self, el, editable):
        if not editable:
            return None
        self._n += 1
        i = str(self._n)
        self.reg[i] = el
        return i

    def scene(self):
        self.reg, self._n, self._warn = {}, 0, []
        ents = []
        for el in kids(self.container):
            n = None
            if el.tag == "model":
                n = self.model_node(el, True, self.res.world_dir)
            elif el.tag == "include":
                n = self.include_node(el, True)
            elif el.tag == "light":
                n = self.light_node(el, True)
            if n:
                ents.append(n)
        seen = set()
        for n in ents:
            base = n["name"] or n["kind"]
            key, i = base, 1
            while key in seen:
                key = f"{base}#{i}"
                i += 1
            seen.add(key)
            n["key"] = key
        world = self.container
        notice, self.notice = self.notice, None
        return dict(file=self.path, world=world.get("name", os.path.basename(self.path)),
                    version=self.version, autosave=self.autosave, dirty=self.dirty,
                    can_undo=bool(self.undo), can_redo=bool(self.redo),
                    entities=ents, warnings=self._warn[:50], notice=notice)

    def model_node(self, el, editable, base_dir):
        node = dict(id=self._reg(el, editable), kind="model", name=el.get("name", ""),
                    pose=parse_pose(el.find("pose")),
                    static=(ftext(el, "static", "false").lower() in ("true", "1")),
                    editable=editable, children=[])
        for c in kids(el):
            if c.tag == "link":
                node["children"].append(self.link_node(c, editable, base_dir))
            elif c.tag == "model":
                node["children"].append(self.model_node(c, editable, base_dir))
            elif c.tag == "include":
                node["children"].append(self.include_node(c, False))
        return node

    def link_node(self, el, editable, base_dir):
        return dict(kind="link", name=el.get("name", ""), pose=parse_pose(el.find("pose")),
                    children=[self.visual_node(v, editable, base_dir) for v in el.findall("visual")])

    def visual_node(self, v, editable, base_dir):
        col = visual_color(v)
        return dict(id=self._reg(v, editable), kind="visual", name=v.get("name", ""),
                    pose=parse_pose(v.find("pose")), geom=self.geom_json(v.find("geometry"), base_dir),
                    color=col or [0.8, 0.8, 0.8, 1.0], color_explicit=col is not None, editable=editable)

    def geom_json(self, g, base_dir):
        if g is None or not kids(g):
            return None
        s = kids(g)[0]
        t = s.tag
        if t == "box":
            return dict(type="box", size=floats(ftext(s, "size", "1 1 1"), 3, [1, 1, 1]))
        if t in ("cylinder", "capsule"):
            return dict(type=t, radius=num(ftext(s, "radius"), 0.5), length=num(ftext(s, "length"), 1))
        if t == "sphere":
            return dict(type="sphere", radius=num(ftext(s, "radius"), 0.5))
        if t == "ellipsoid":
            return dict(type="ellipsoid", radii=floats(ftext(s, "radii", "1 1 1"), 3, [1, 1, 1]))
        if t == "plane":
            return dict(type="plane", normal=floats(ftext(s, "normal", "0 0 1"), 3, [0, 0, 1]),
                        size=floats(ftext(s, "size", "1 1"), 2, [1, 1]))
        if t == "mesh":
            uri = ftext(s, "uri", "")
            scale = floats(ftext(s, "scale", "1 1 1"), 3, [1, 1, 1])
            p = self.res.resolve_file(uri, base_dir) if uri else None
            if p:
                return dict(type="mesh", uri=uri, url=file_url(p), ext=os.path.splitext(p)[1].lower().lstrip("."),
                            scale=scale)
            self._warn.append(f"Mesh not found: {uri}")
            return dict(type="placeholder", uri=uri, scale=scale)
        return dict(type=t)

    def include_node(self, el, editable):
        uri = ftext(el, "uri", "")
        md, sroot = self.res.load_model(uri)
        pe = el.find("pose")
        node = dict(id=self._reg(el, editable), kind="include", uri=uri, resolved=sroot is not None,
                    editable=False, children=[])
        inner = None
        if sroot is not None:
            if sroot.tag in ("model", "light"):
                inner = sroot
            else:
                inner = sroot.find("model")
                if inner is None:
                    inner = sroot.find("light")
        name = ftext(el, "name")
        default_pose = [0.0] * 6
        static = ftext(el, "static")
        if inner is not None and inner.tag == "model":
            m = self.model_node(inner, False, md)
            node["children"] = m["children"]
            name = name or m["name"]
            default_pose = m["pose"]
            if static is None:
                static = "true" if m["static"] else "false"
        elif inner is not None and inner.tag == "light":
            ln = self.light_node(inner, False)
            name = name or ln["name"]
            default_pose = ln["pose"]
            ln["pose"] = [0.0] * 6
            node["children"] = [ln]
        else:
            low = uri.lower()
            node["resolved"] = False
            if "ground" in low and "plane" in low:
                node["children"] = [dict(kind="visual", name="ground", pose=[0.0] * 6, editable=False,
                                         geom=dict(type="plane", normal=[0, 0, 1], size=[100, 100]),
                                         color=[0.7, 0.7, 0.7, 1], color_explicit=True)]
                static = static or "true"
            elif low.rstrip("/").endswith("sun"):
                node["children"] = [dict(kind="light", name="sun", light_type="directional",
                                         direction=[-0.5, 0.1, -0.9], pose=[0.0] * 6)]
                default_pose = [0, 0, 10, 0, 0, 0]
            else:
                node["children"] = [dict(kind="visual", name="missing", pose=[0.0] * 6, editable=False,
                                         geom=dict(type="placeholder", uri=uri), color=[1, 0, 0, 1],
                                         color_explicit=True)]
                self._warn.append(f"Model not found: {uri} (add its folder with --model-path)")
        node["name"] = name or os.path.basename(uri.rstrip("/")) or "include"
        node["pose"] = parse_pose(pe) if pe is not None else default_pose
        node["static"] = (static or "false").lower() in ("true", "1")
        return node

    def light_node(self, el, editable):
        return dict(id=self._reg(el, editable), kind="light", name=el.get("name", ""),
                    pose=parse_pose(el.find("pose")), light_type=el.get("type", "point"),
                    direction=floats(ftext(el, "direction", ""), 3),
                    color=floats(ftext(el, "diffuse", ""), 3))

    # ---- edit operations
    def _get(self, data, tags=None):
        el = self.reg.get(str(data.get("id")))
        if el is None:
            raise ValueError("That element no longer exists - the view was refreshed, try again.")
        if tags and el.tag not in tags:
            raise ValueError(f"Can't do that on <{el.tag}>")
        return el

    def _top_names(self, exclude=None):
        names = set()
        for c in kids(self.container):
            if c is exclude:
                continue
            n = c.get("name") or ftext(c, "name")
            if n:
                names.add(n)
        return names

    def _child(self, el, tag, index=None):
        c = el.find(tag)
        if c is None:
            c = ET.Element(tag)
            insert_pretty(el, c, index, self.depth_of(el))
        return c

    def op(self, data):
        name = data.get("op", "")
        with self.lock:
            if name == "undo" or name == "redo":
                src, dst = (self.undo, self.redo) if name == "undo" else (self.redo, self.undo)
                if not src:
                    raise ValueError(f"Nothing to {name}")
                dst.append(self.serialize())
                self._load_text(src.pop())
                self._changed()
                return {}
            if name == "reload":
                before = self.serialize()
                self.load_from_disk()
                self.undo.append(before)
                self.res.cache.clear()
                self.version += 1
                return {}
            if name == "save":
                self.save()
                self.version += 1
                return {}
            if name == "set_autosave":
                self.autosave = bool(data.get("value"))
                if self.autosave and self.dirty:
                    self.save()
                self.version += 1
                return {}
            fn = getattr(self, "op_" + name, None)
            if fn is None:
                raise ValueError(f"Unknown operation {name}")
            before = self.serialize()
            try:
                result = fn(data) or {}
            except Exception:
                self._load_text(before)
                raise
            self.undo.append(before)
            del self.undo[:-100]
            self.redo.clear()
            self._changed()
            return result

    def _changed(self):
        self.version += 1
        self.dirty = True
        if self.autosave:
            self.save()

    def op_set_pose(self, d):
        el = self._get(d, ("model", "include", "light", "visual"))
        pose = [float(x) for x in d["pose"]]
        if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
            raise ValueError("Pose needs 6 numbers")
        write_pose(self._child(el, "pose", 0), pose)

    def op_set_name(self, d):
        el = self._get(d, ("model", "include", "light", "visual"))
        new = str(d.get("name", "")).strip()
        if not new or not re.match(r"^[^\s<>&\"']+$", new):
            raise ValueError("Name must be non-empty without spaces or < > & quotes")
        if self.parent_of(el) is self.container and new in self._top_names(exclude=el):
            raise ValueError(f"'{new}' is already used in this world")
        if el.tag == "include":
            self._child(el, "name", 0).text = new
        else:
            el.set("name", new)
        return {"select": new}

    def op_set_static(self, d):
        el = self._get(d, ("model", "include"))
        self._child(el, "static", 0).text = "true" if d.get("value") else "false"

    def op_set_geom(self, d):
        vis = self._get(d, ("visual",))
        g = vis.find("geometry")
        if g is None or not kids(g):
            raise ValueError("This visual has no geometry")
        shape = kids(g)[0]
        allowed = GEOM_KEYS.get(shape.tag, {})
        new = {}
        for k, v in (d.get("params") or {}).items():
            if k not in allowed:
                raise ValueError(f"{shape.tag} has no '{k}'")
            vals = v if isinstance(v, list) else [v]
            vals = [float(x) for x in vals]
            if len(vals) != allowed[k] or not all(math.isfinite(x) for x in vals):
                raise ValueError(f"'{k}' needs {allowed[k]} number(s)")
            if k != "normal" and any(x <= 0 for x in vals):
                raise ValueError(f"'{k}' must be greater than 0")
            new[k] = fmt_list(vals)
        old = {k: " ".join((shape.findtext(k) or "").split()) for k in new}
        for k, v in new.items():
            self._child(shape, k).text = v
        # keep matching collisions of the same link in sync
        link = self.parent_of(vis)
        if link is not None:
            for col in link.findall("collision"):
                cg = col.find("geometry")
                cs = kids(cg)[0] if cg is not None and kids(cg) else None
                if cs is None or cs.tag != shape.tag or shape.tag == "mesh":
                    continue
                if all(" ".join((cs.findtext(k) or "").split()) == old[k] for k in new):
                    for k, v in new.items():
                        self._child(cs, k).text = v

    def op_set_color(self, d):
        vis = self._get(d, ("visual",))
        rgba = [min(1.0, max(0.0, float(x))) for x in d["rgba"]][:4]
        if len(rgba) == 3:
            rgba.append(1.0)
        m = self._child(vis, "material")
        for s in m.findall("script"):
            remove_pretty(m, s)
        self._child(m, "ambient").text = fmt_list(rgba)
        self._child(m, "diffuse").text = fmt_list(rgba)

    def op_add(self, d):
        shape = d.get("shape")
        if shape not in GEOM_XML:
            raise ValueError("Shape must be box, cylinder or sphere")
        names = self._top_names()
        i = 1
        while f"{shape}_{i}" in names:
            i += 1
        name = f"{shape}_{i}"
        x, y = (list(d.get("position") or [0, 0]) + [0, 0])[:2]
        ixx, iyy, izz = INERTIA[shape]
        geo, col = GEOM_XML[shape], NEW_COLOR[shape]
        xml = (f'<model name="{name}"><static>true</static><pose>{fmt_list([x, y, 0.5, 0, 0, 0])}</pose>'
               f'<link name="link"><inertial><mass>1</mass><inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>'
               f'<iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia></inertial>'
               f'<collision name="collision"><geometry>{geo}</geometry></collision>'
               f'<visual name="visual"><geometry>{geo}</geometry>'
               f'<material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>'
               f'</link></model>')
        insert_pretty(self.container, parse_fragment(xml), None, self.depth_of(self.container))
        return {"select": name}

    def op_delete(self, d):
        el = self._get(d)
        parent = self.parent_of(el)
        if parent is not self.container:
            raise ValueError("Only top-level models, includes and lights can be deleted here")
        remove_pretty(parent, el)
        return {"select": ""}

    def op_duplicate(self, d):
        el = self._get(d, ("model", "include", "light"))
        parent = self.parent_of(el)
        if parent is not self.container:
            raise ValueError("Only top-level entities can be duplicated")
        ks = list(parent)
        i = ks.index(el)
        before_ws = parent.text if i == 0 else ks[i - 1].tail
        new = copy.deepcopy(el)
        new.tail = el.tail
        el.tail = before_ws if _ws(before_ws) is not None else "\n"
        parent.insert(i + 1, new)
        old_name = el.get("name") or ftext(el, "name") or os.path.basename((ftext(el, "uri") or "copy").rstrip("/"))
        base = re.sub(r"_\d+$", "", old_name)
        names, n = self._top_names(), 1
        while f"{base}_{n}" in names:
            n += 1
        name = f"{base}_{n}"
        if new.tag == "include":
            self._child(new, "name", 0).text = name
        else:
            new.set("name", name)
        pose = parse_pose(new.find("pose"))
        pose[0] += 1.0
        write_pose(self._child(new, "pose", 0), pose)
        return {"select": name}

    def get_xml(self, d):
        el = self._get(d)
        tail, el.tail = el.tail, None
        try:
            s = ET.tostring(el, encoding="unicode")
        finally:
            el.tail = tail
        parent = self.parent_of(el)
        if parent is not None:
            inner, _, _ = _layout(parent, self.depth_of(parent))
            base = inner.lstrip("\n")
            s = "\n".join(l[len(base):] if l.startswith(base) else l for l in s.split("\n"))
        return s

    def op_set_xml(self, d):
        el = self._get(d)
        try:
            new = parse_fragment(d.get("xml", ""))
        except ET.ParseError as e:
            raise ValueError(f"XML error: {e}")
        parent = self.parent_of(el)
        inner, _, unit = _layout(parent, self.depth_of(parent))
        _reindent(new, inner, unit)
        new.tail = el.tail
        ks = list(parent)
        parent[ks.index(el)] = new
        n = new.get("name") or ftext(new, "name")
        return {"select": n} if n else {}


# --------------------------------------------------------------------------- web server
def make_handler(doc):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json", cache=False):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=60" if cache else "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path in ("/", "/index.html"):
                return self._send(200, PAGE, "text/html; charset=utf-8")
            if u.path == "/api/scene":
                with doc.lock:
                    doc.check_external()
                    return self._send(200, doc.scene())
            if u.path == "/api/version":
                with doc.lock:
                    doc.check_external()
                    return self._send(200, {"version": doc.version})
            if u.path == "/api/xml":
                with doc.lock:
                    try:
                        q = parse_qs(u.query)
                        return self._send(200, {"xml": doc.get_xml({"id": q.get("id", [""])[0]})})
                    except ValueError as e:
                        return self._send(400, {"error": str(e)})
            if u.path.startswith("/file/"):
                p = unquote(u.path[5:])
                if re.match(r"^/[A-Za-z]:/", p):
                    p = p[1:]
                p = os.path.normpath(p)
                if not os.path.isfile(p) or not doc.res.is_allowed(p):
                    return self._send(404, {"error": "not found"})
                ctype = mimetypes.guess_type(p)[0] or "application/octet-stream"
                with open(p, "rb") as f:
                    return self._send(200, f.read(), ctype, cache=True)
            return self._send(404, {"error": "not found"})

        def do_POST(self):
            if urlparse(self.path).path != "/api/op":
                return self._send(404, {"error": "not found"})
            try:
                n = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, json.JSONDecodeError):
                return self._send(400, {"error": "bad request"})
            with doc.lock:
                try:
                    result = doc.op(data)
                except (ValueError, KeyError, TypeError) as e:
                    return self._send(200, {"error": str(e), "scene": doc.scene()})
                except OSError as e:
                    return self._send(200, {"error": f"Could not write the file: {e}", "scene": doc.scene()})
                result["scene"] = doc.scene()
                return self._send(200, result)

    return Handler


def main():
    ap = argparse.ArgumentParser(description="View and edit a Gazebo world; edits are written back to the SDF.")
    ap.add_argument("world", help="path to a .world or .sdf file (created if it doesn't exist)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--model-path", action="append", default=[],
                    help="extra folder containing models (for model:// URIs); can repeat")
    ap.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    args = ap.parse_args()

    path = os.path.abspath(os.path.expanduser(args.world))
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(BLANK_WORLD)
        print(f"Created a new blank world: {path}")

    res = Resolver(path, args.model_path)
    try:
        doc = WorldDoc(path, res)
    except ET.ParseError as e:
        sys.exit(f"Could not parse {path}: {e}")

    srv = None
    for port in range(args.port, args.port + 20):
        try:
            srv = ThreadingHTTPServer((args.host, port), make_handler(doc))
            break
        except OSError:
            continue
    if srv is None:
        sys.exit("No free port found")
    url = f"http://{'localhost' if args.host in ('127.0.0.1', '0.0.0.0') else args.host}:{srv.server_port}/"
    print(f"Editing  {doc.path}")
    print(f"Backup   {doc.path}.bak")
    print(f"Model search paths: {len(res.model_paths)}")
    for p in res.model_paths:
        print(f"   - {p}")
    print(f"\nOpen  {url}   (Ctrl+C to quit)")
    if not args.no_browser:
        threading.Timer(0.7, webbrowser.open, [url]).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nBye")


# --------------------------------------------------------------------------- browser UI
PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>SDF World Editor</title>
<style>
*{box-sizing:border-box}
html,body{margin:0;height:100%;font:13px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;color:#e6e8eb;background:#1b1e23}
#app{display:grid;grid-template-columns:230px 1fr 310px;grid-template-rows:46px 1fr;height:100vh}
#top{grid-column:1/4;display:flex;align-items:center;gap:8px;padding:0 10px;background:#23272e;border-bottom:1px solid #333a44}
#left,#right{background:#21252b;overflow:auto;padding:10px}
#left{border-right:1px solid #333a44}#right{border-left:1px solid #333a44}
#view{position:relative;overflow:hidden}
button{background:#2f3540;color:#e6e8eb;border:1px solid #434b58;border-radius:6px;padding:5px 9px;cursor:pointer;font:inherit}
button:hover{background:#38404c}button.on{background:#2d5b8a;border-color:#3d76b0}button:disabled{opacity:.4;cursor:default}
input,textarea{background:#16191d;color:#e6e8eb;border:1px solid #3a414c;border-radius:5px;padding:4px 6px;font:inherit;width:100%}
input[type=checkbox]{width:auto;vertical-align:middle}
input[type=color]{padding:0;height:26px;width:44px;flex:none}
.title{font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:34vw;direction:rtl;text-align:left}
.sp{flex:1}
.pill{padding:3px 10px;border-radius:999px;font-size:12px;background:#24402a;color:#9fe39f;white-space:nowrap}
.pill.dirty{background:#4a3b1f;color:#ffd27a}.pill.err{background:#4a2323;color:#ff9c9c}
h4{margin:14px 0 6px;font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:#8d96a3}
.item{padding:5px 7px;border-radius:5px;cursor:pointer;display:flex;gap:6px;align-items:center;white-space:nowrap;overflow:hidden}
.item:hover{background:#2a2f37}.item.sel{background:#2d5b8a}
.k{font-size:10px;padding:1px 5px;border-radius:4px;background:#39414d;color:#b9c2cf;flex:none}
.row{display:grid;grid-template-columns:repeat(3,1fr);gap:5px;margin-bottom:5px}
label.f{display:block;font-size:11px;color:#8d96a3;margin-bottom:2px}
.vis{border:1px solid #333a44;border-radius:7px;padding:8px;margin-bottom:8px}
.btns{display:flex;gap:6px;flex-wrap:wrap}
textarea{min-height:260px;font:12px/1.45 ui-monospace,Menlo,Consolas,monospace;white-space:pre;margin-top:6px}
#toast{position:absolute;left:50%;bottom:16px;transform:translateX(-50%);background:#0d0f12;color:#fff;padding:8px 14px;border-radius:8px;opacity:0;transition:opacity .25s;pointer-events:none;max-width:80%}
.muted{color:#8d96a3}.warn{color:#ffd27a;font-size:12px;margin-top:4px;word-break:break-all}
label{white-space:nowrap}
</style>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<div id="app">
 <div id="top">
  <span class="title" id="file"></span><span class="pill" id="status">Loading…</span>
  <span class="sp"></span>
  <button id="bMove" class="on" title="Move (W)">Move</button><button id="bRot" title="Rotate (E)">Rotate</button>
  <button id="bSpace" title="Gizmo axes: world or local">World axes</button>
  <label title="Snap 0.1 m / 15°"><input type="checkbox" id="snap"> Snap</label>
  <button id="bUndo" title="Undo (Ctrl+Z)">Undo</button><button id="bRedo" title="Redo (Ctrl+Y)">Redo</button>
  <label title="Write every edit to the SDF file immediately"><input type="checkbox" id="auto" checked> Live save</label>
  <button id="bSave" title="Save (Ctrl+S)">Save</button><button id="bReload" title="Re-read the file from disk">Reload</button>
 </div>
 <div id="left">
  <h4>Add</h4><div class="btns"><button data-add="box">+ Box</button><button data-add="cylinder">+ Cylinder</button><button data-add="sphere">+ Sphere</button></div>
  <h4>World</h4><input id="filter" placeholder="Filter…"><div id="list" style="margin-top:6px"></div>
  <div id="warns"></div>
 </div>
 <div id="view"><div id="toast"></div></div>
 <div id="right"><div id="props"></div></div>
</div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {TransformControls} from 'three/addons/controls/TransformControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {ColladaLoader} from 'three/addons/loaders/ColladaLoader.js';
import {OBJLoader} from 'three/addons/loaders/OBJLoader.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';

const $ = s => document.querySelector(s);
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const D2R = Math.PI/180, R2D = 180/Math.PI;
const r4 = v => Number.isFinite(v) ? Math.round(v*1e4)/1e4 : 0;
const KIND = {model:'model', include:'include', light:'light'};

let S = null, version = -1, selected = null, groups = new Map(), selBox = null, firstLoad = true;

// ---------- three.js scene (Gazebo is Z-up)
THREE.Object3D.DEFAULT_UP.set(0,0,1);
const view = $('#view');
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
view.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xb9c3cd);
const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 5000);
camera.up.set(0,0,1); camera.position.set(9,-9,7);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
const hemi = new THREE.HemisphereLight(0xffffff, 0x555560, 1.1); hemi.position.set(0,0,1); scene.add(hemi);
const key = new THREE.DirectionalLight(0xffffff, 1.4); key.position.set(6,-4,10); scene.add(key);
const grid = new THREE.GridHelper(50, 50, 0x555555, 0x8a8f96); grid.rotation.x = Math.PI/2; grid.position.z = 0.002; scene.add(grid);
scene.add(new THREE.AxesHelper(1));
const worldRoot = new THREE.Group(); scene.add(worldRoot);
const tc = new TransformControls(camera, renderer.domElement);
tc.setSize(0.9); scene.add(tc);

function resize(){ const w=view.clientWidth, h=view.clientHeight; renderer.setSize(w,h); camera.aspect=w/Math.max(h,1); camera.updateProjectionMatrix(); }
new ResizeObserver(resize).observe(view);
renderer.setAnimationLoop(()=>{ orbit.update(); if(selBox) selBox.update(); renderer.render(scene, camera); });

// ---------- meshes
const meshCache = new Map();
function loadMesh(url, ext){
  if(!meshCache.has(url)){
    meshCache.set(url, new Promise((res, rej)=>{
      if(ext==='stl') new STLLoader().load(url, geo=>{ geo.computeVertexNormals(); res(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({color:0xbbbbbb, roughness:.7}))); }, undefined, rej);
      else if(ext==='dae') new ColladaLoader().load(url, c=>{ const w=new THREE.Group(); w.rotation.x=Math.PI/2; w.add(c.scene); res(w); }, undefined, rej);
      else if(ext==='obj') new OBJLoader().load(url, o=>res(o), undefined, rej);
      else if(ext==='glb'||ext==='gltf') new GLTFLoader().load(url, g=>{ const w=new THREE.Group(); w.rotation.x=Math.PI/2; w.add(g.scene); res(w); }, undefined, rej);
      else rej(new Error('Unsupported mesh type .'+ext));
    }));
  }
  return meshCache.get(url).then(o=>o.clone(true));
}

function mat(c){
  const [r,g,b,a] = c || [.8,.8,.8,1];
  const m = new THREE.MeshStandardMaterial({roughness:.75, metalness:.05, side:THREE.DoubleSide, transparent:(a??1)<1, opacity:a??1});
  m.color.setRGB(r,g,b, THREE.SRGBColorSpace);
  return m;
}
function applyPose(o,p){ p=p||[0,0,0,0,0,0]; o.position.set(p[0],p[1],p[2]); o.rotation.set(p[3],p[4],p[5],'ZYX'); }

function placeholder(){ return new THREE.Mesh(new THREE.BoxGeometry(.5,.5,.5), new THREE.MeshBasicMaterial({color:0xff3333, wireframe:true})); }

function addVisual(g, n){
  const geo = n.geom; if(!geo) return;
  let m = null;
  switch(geo.type){
    case 'box': m = new THREE.Mesh(new THREE.BoxGeometry(...geo.size), mat(n.color)); break;
    case 'cylinder': m = new THREE.Mesh(new THREE.CylinderGeometry(geo.radius, geo.radius, geo.length, 40), mat(n.color)); m.rotation.x=Math.PI/2; break;
    case 'capsule': m = new THREE.Mesh(new THREE.CapsuleGeometry(geo.radius, geo.length, 8, 24), mat(n.color)); m.rotation.x=Math.PI/2; break;
    case 'sphere': m = new THREE.Mesh(new THREE.SphereGeometry(geo.radius, 40, 20), mat(n.color)); break;
    case 'ellipsoid': m = new THREE.Mesh(new THREE.SphereGeometry(1, 40, 20), mat(n.color)); m.scale.set(...geo.radii); break;
    case 'plane': {
      m = new THREE.Mesh(new THREE.PlaneGeometry(geo.size[0], geo.size[1]), mat(n.color));
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0,0,1), new THREE.Vector3(...geo.normal).normalize());
      m.userData.isPlane = true; break; }
    case 'mesh': {
      const holder = new THREE.Group(); holder.scale.set(...(geo.scale||[1,1,1])); g.add(holder);
      loadMesh(geo.url, geo.ext).then(obj=>{
        if(n.color_explicit) obj.traverse(o=>{ if(o.isMesh) o.material = mat(n.color); });
        holder.add(obj);
      }).catch(err=>{ console.warn(geo.uri, err); holder.add(placeholder()); toast('Could not load mesh '+geo.uri); });
      return; }
    case 'placeholder': m = placeholder(); break;
    default: return;
  }
  g.add(m);
}

function addLight(g, n){
  g.add(new THREE.Mesh(new THREE.SphereGeometry(.15, 16, 8), new THREE.MeshBasicMaterial({color:0xffcc33})));
  if(n.direction){ const d=new THREE.Vector3(...n.direction); if(d.length()>0) g.add(new THREE.ArrowHelper(d.normalize(), new THREE.Vector3(), 1.2, 0xff9900, .25, .15)); }
}

function buildNode(n, parent, top){
  const g = new THREE.Group(); g.rotation.order = 'ZYX'; applyPose(g, n.pose); g.userData.top = top;
  if(n.kind==='visual') addVisual(g, n);
  else if(n.kind==='light') addLight(g, n);
  for(const c of n.children||[]) buildNode(c, g, top);
  parent.add(g); return g;
}

function rebuild(){
  tc.detach();
  worldRoot.clear(); groups.clear();
  for(const e of S.entities) groups.set(e.key, buildNode(e, worldRoot, e.key));
  renderList(); select(selected);
}

// ---------- selection & picking
const entity = k => S?.entities.find(e=>e.key===k);
function select(k){
  selected = k && groups.has(k) ? k : null;
  const g = selected ? groups.get(selected) : null, e = entity(selected);
  if(g && e && e.id) tc.attach(g); else tc.detach();
  if(selBox){ scene.remove(selBox); selBox=null; }
  if(g){ selBox = new THREE.BoxHelper(g, 0xffd400); scene.add(selBox); }
  document.querySelectorAll('#list .item').forEach(n=>n.classList.toggle('sel', n.dataset.key===selected));
  renderProps();
}
const ray = new THREE.Raycaster(); let down = null;
renderer.domElement.addEventListener('pointerdown', e=>{ down=[e.clientX,e.clientY]; });
renderer.domElement.addEventListener('pointerup', e=>{
  if(!down) return; const moved = Math.hypot(e.clientX-down[0], e.clientY-down[1]); down=null;
  if(moved>4 || tc.dragging || tc.axis) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((e.clientX-r.left)/r.width)*2-1, -((e.clientY-r.top)/r.height)*2+1), camera);
  const hits = ray.intersectObjects(worldRoot.children, true).filter(h=>h.object.isMesh);
  const h = hits.find(h=>!h.object.userData.isPlane) || hits[0];
  let o = h ? h.object : null; while(o && o.userData.top===undefined) o = o.parent;
  select(o ? o.userData.top : null);
});

tc.addEventListener('dragging-changed', ev=>{ orbit.enabled = !ev.value; if(!ev.value) commitGizmo(); });
tc.addEventListener('objectChange', ()=>{
  const g = groups.get(selected); if(!g) return;
  const v = [g.position.x, g.position.y, g.position.z, g.rotation.x*R2D, g.rotation.y*R2D, g.rotation.z*R2D];
  document.querySelectorAll('[data-pose]').forEach(i=>i.value = r4(v[+i.dataset.pose]));
});
function commitGizmo(){
  const e = entity(selected), g = groups.get(selected); if(!e || !g || !e.id) return;
  const pose = [g.position.x, g.position.y, g.position.z, g.rotation.x, g.rotation.y, g.rotation.z];
  if(pose.every((v,i)=>Math.abs(v-e.pose[i])<1e-7)) return;
  op('set_pose', {id:e.id, pose});
}

// ---------- camera helpers
function fit(box){
  const c = box.getCenter(new THREE.Vector3()), s = box.getSize(new THREE.Vector3()).length() || 2;
  const dir = camera.position.clone().sub(orbit.target).normalize();
  orbit.target.copy(c); camera.position.copy(c).addScaledVector(dir, s*1.1 + 1);
  camera.near = Math.max(0.01, s/2000); camera.far = Math.max(2000, s*30); camera.updateProjectionMatrix();
}
function frameAll(){
  const box = new THREE.Box3(); worldRoot.updateMatrixWorld(true);
  worldRoot.traverse(o=>{ if(o.isMesh && !o.userData.isPlane) box.expandByObject(o); });
  if(!box.isEmpty()) fit(box);
}
function focusSel(){ const g = groups.get(selected); if(g){ const b = new THREE.Box3().setFromObject(g); if(!b.isEmpty()) fit(b); } }

// ---------- UI
function setStatus(text, cls=''){ const p=$('#status'); p.textContent=text; p.className='pill '+cls; }
let toastT; function toast(msg){ const t=$('#toast'); t.textContent=msg; t.style.opacity=1; clearTimeout(toastT); toastT=setTimeout(()=>t.style.opacity=0, 3500); }

function renderList(){
  const f = $('#filter').value.toLowerCase();
  $('#list').innerHTML = S.entities.filter(e=>!f || e.key.toLowerCase().includes(f))
    .map(e=>`<div class="item${e.key===selected?' sel':''}" data-key="${esc(e.key)}"><span class="k">${KIND[e.kind]||e.kind}</span><span>${esc(e.key)}</span></div>`).join('')
    || '<div class="muted">Nothing here yet</div>';
  const w = S.warnings||[];
  $('#warns').innerHTML = w.length ? '<h4>Warnings</h4>' + w.map(x=>`<div class="warn">${esc(x)}</div>`).join('') : '';
}
$('#list').addEventListener('click', e=>{ const it=e.target.closest('.item'); if(it) select(it.dataset.key); });
$('#filter').addEventListener('input', renderList);

function collectVisuals(n, out){ for(const c of n.children||[]){ if(c.kind==='visual') out.push(c); collectVisuals(c, out); } return out; }

function visualEditor(v){
  const g = v.geom || {}, dis = v.editable ? '' : 'disabled';
  const field = (label, key, idx, val, step=0.05) => `<div><label class="f">${label}</label><input type="number" step="${step}" data-vid="${v.id}" data-key="${key}" ${idx!=null?`data-idx="${idx}"`:''} value="${r4(val)}" ${dis}></div>`;
  let f = '';
  if(g.type==='box') f = `<div class="row">${field('size x','size',0,g.size[0])}${field('size y','size',1,g.size[1])}${field('size z','size',2,g.size[2])}</div>`;
  else if(g.type==='cylinder'||g.type==='capsule') f = `<div class="row">${field('radius','radius',null,g.radius)}${field('length','length',null,g.length)}</div>`;
  else if(g.type==='sphere') f = `<div class="row">${field('radius','radius',null,g.radius)}</div>`;
  else if(g.type==='ellipsoid') f = `<div class="row">${field('rx','radii',0,g.radii[0])}${field('ry','radii',1,g.radii[1])}${field('rz','radii',2,g.radii[2])}</div>`;
  else if(g.type==='plane') f = `<div class="row">${field('size x','size',0,g.size[0],1)}${field('size y','size',1,g.size[1],1)}</div>`;
  else if(g.type==='mesh'||g.type==='placeholder') f = `<div class="muted" style="word-break:break-all;margin-bottom:4px">${esc(g.uri||'')}</div>` + (g.scale ? `<div class="row">${field('scale x','scale',0,g.scale[0],.1)}${field('scale y','scale',1,g.scale[1],.1)}${field('scale z','scale',2,g.scale[2],.1)}</div>` : '');
  else f = `<div class="muted">${esc(g.type||'no geometry')} (edit via XML)</div>`;
  const c = v.color || [.8,.8,.8,1];
  const hex = '#' + c.slice(0,3).map(x=>Math.round(Math.min(1,Math.max(0,x))*255).toString(16).padStart(2,'0')).join('');
  return `<div class="vis"><div style="display:flex;justify-content:space-between;margin-bottom:6px"><b>${esc(v.name||'visual')}</b><span class="k">${esc(g.type||'')}</span></div>${f}
    <div style="display:flex;gap:8px;align-items:center;margin-top:4px"><input type="color" data-color="${v.id}" value="${hex}" ${dis}><label class="f" style="margin:0">alpha</label><input type="number" min="0" max="1" step="0.1" data-alpha="${v.id}" value="${r4(c[3]??1)}" style="width:72px" ${dis}></div></div>`;
}

function renderProps(){
  const p = $('#props'), e = entity(selected);
  if(!e){
    p.innerHTML = `<h4>World</h4><div><b>${esc(S?.world||'')}</b></div><div class="muted">${S?S.entities.length:0} top-level entities</div>
      <h4>How to edit</h4><div class="muted">Click something to select it, then drag the gizmo or type exact values here.
      With <b>Live save</b> on, every change goes straight into the .world file (the original is kept as <code>.bak</code>).<br><br>
      W move · E rotate · F focus · Del delete<br>Ctrl+D duplicate · Ctrl+Z undo · Ctrl+Y redo</div>`;
    return;
  }
  const ro = !e.id, dis = ro ? 'disabled' : '';
  let h = `<h4>${esc(e.kind)}</h4><label class="f">Name</label><input id="pName" value="${esc(e.name)}" ${dis}>`;
  if(e.kind!=='light') h += `<label style="display:block;margin-top:8px"><input type="checkbox" id="pStatic" ${e.static?'checked':''} ${dis}> Static</label>`;
  if(e.kind==='include') h += `<label class="f" style="margin-top:8px">URI</label><input value="${esc(e.uri)}" disabled>` + (e.resolved ? '' : '<div class="warn">Model files not found, so a stand-in is shown. Pass its folder with --model-path.</div>');
  h += `<h4>Pose</h4>
    <div class="row">${['x','y','z'].map((a,i)=>`<div><label class="f">${a} (m)</label><input type="number" step="0.05" data-pose="${i}" value="${r4(e.pose[i])}" ${dis}></div>`).join('')}</div>
    <div class="row">${['roll','pitch','yaw'].map((a,i)=>`<div><label class="f">${a} (°)</label><input type="number" step="5" data-pose="${i+3}" value="${r4(e.pose[i+3]*R2D)}" ${dis}></div>`).join('')}</div>`;
  const vis = collectVisuals(e, []);
  if(vis.length){
    h += `<h4>Visuals${vis[0].editable ? '' : ' · from included model (read-only)'}</h4>` + vis.slice(0,30).map(visualEditor).join('');
    if(vis.length>30) h += `<div class="muted">+${vis.length-30} more (use Edit XML)</div>`;
  }
  if(e.id) h += `<h4>Actions</h4><div class="btns"><button id="aFocus">Focus</button><button id="aDup">Duplicate</button><button id="aDel">Delete</button><button id="aXml">Edit XML</button></div><div id="xmlBox"></div>`;
  p.innerHTML = h;
  if(ro) return;

  $('#pName').onchange = ev => op('set_name', {id:e.id, name:ev.target.value});
  const st = $('#pStatic'); if(st) st.onchange = ev => op('set_static', {id:e.id, value:ev.target.checked});
  p.querySelectorAll('[data-pose]').forEach(inp => inp.onchange = () => {
    const k = +inp.dataset.pose, v = parseFloat(inp.value); if(!Number.isFinite(v)) return;
    const pose = [...e.pose]; pose[k] = k>2 ? v*D2R : v; op('set_pose', {id:e.id, pose});
  });
  const findV = id => vis.find(v=>v.id===id);
  p.querySelectorAll('[data-vid]').forEach(inp => inp.onchange = () => {
    const v = findV(inp.dataset.vid), key = inp.dataset.key, n = parseFloat(inp.value);
    if(!(n>0)) return toast('Must be greater than 0');
    let val = n; if(inp.dataset.idx!=null){ val = [...v.geom[key]]; val[+inp.dataset.idx] = n; }
    op('set_geom', {id:v.id, params:{[key]:val}});
  });
  const sendColor = id => {
    const hex = p.querySelector(`[data-color="${id}"]`).value, a = parseFloat(p.querySelector(`[data-alpha="${id}"]`).value);
    const c = new THREE.Color(); c.setStyle(hex, THREE.SRGBColorSpace); const o = {}; c.getRGB(o, THREE.SRGBColorSpace);
    op('set_color', {id, rgba:[o.r, o.g, o.b, Number.isFinite(a)?a:1]});
  };
  p.querySelectorAll('[data-color]').forEach(i => i.onchange = () => sendColor(i.dataset.color));
  p.querySelectorAll('[data-alpha]').forEach(i => i.onchange = () => sendColor(i.dataset.alpha));
  $('#aFocus').onclick = focusSel;
  $('#aDup').onclick = () => op('duplicate', {id:e.id});
  $('#aDel').onclick = () => op('delete', {id:e.id});
  $('#aXml').onclick = async () => {
    const j = await (await fetch('/api/xml?id='+encodeURIComponent(e.id))).json();
    if(j.error) return toast(j.error);
    $('#xmlBox').innerHTML = `<textarea id="xmlText" spellcheck="false"></textarea><div class="btns" style="margin-top:6px"><button id="xApply" class="on">Apply XML</button><button id="xCancel">Cancel</button></div>`;
    $('#xmlText').value = j.xml;
    $('#xApply').onclick = () => op('set_xml', {id:e.id, xml:$('#xmlText').value});
    $('#xCancel').onclick = () => $('#xmlBox').innerHTML = '';
  };
}

// ---------- server calls
async function op(name, args={}){
  setStatus('Saving…', 'dirty');
  try{
    const r = await fetch('/api/op', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({op:name, ...args})});
    const j = await r.json();
    if(j.error) toast(j.error);
    if(j.scene) applyScene(j.scene, j.select);
  }catch(err){ setStatus('Offline', 'err'); toast('Cannot reach the Python app. Is it still running?'); }
}
function applyScene(sc, sel){
  S = sc; version = sc.version;
  if(sel !== undefined && sel !== null) selected = sel || null;
  $('#file').textContent = sc.file; $('#file').title = sc.file;
  $('#auto').checked = sc.autosave;
  $('#bSave').disabled = !sc.dirty; $('#bUndo').disabled = !sc.can_undo; $('#bRedo').disabled = !sc.can_redo;
  setStatus(sc.dirty ? 'Unsaved changes' : 'Saved to file', sc.dirty ? 'dirty' : '');
  if(sc.notice) toast(sc.notice);
  rebuild();
  if(firstLoad){ firstLoad = false; frameAll(); }
}
setInterval(async () => {
  if(tc.dragging) return;
  try{
    const j = await (await fetch('/api/version')).json();
    if(j.version !== version) applyScene(await (await fetch('/api/scene')).json());
  }catch{ setStatus('Offline', 'err'); }
}, 1500);

// ---------- toolbar & keys
function setMode(m){ tc.setMode(m); $('#bMove').classList.toggle('on', m==='translate'); $('#bRot').classList.toggle('on', m==='rotate'); }
$('#bMove').onclick = () => setMode('translate');
$('#bRot').onclick = () => setMode('rotate');
$('#bSpace').onclick = () => { const s = tc.space==='world' ? 'local' : 'world'; tc.setSpace(s); $('#bSpace').textContent = s==='world' ? 'World axes' : 'Local axes'; };
$('#snap').onchange = ev => { tc.setTranslationSnap(ev.target.checked ? 0.1 : null); tc.setRotationSnap(ev.target.checked ? 15*D2R : null); };
$('#bUndo').onclick = () => op('undo');
$('#bRedo').onclick = () => op('redo');
$('#auto').onchange = ev => op('set_autosave', {value:ev.target.checked});
$('#bSave').onclick = () => op('save');
$('#bReload').onclick = () => { meshCache.clear(); op('reload'); };
document.querySelectorAll('[data-add]').forEach(b => b.onclick = () => op('add', {shape:b.dataset.add, position:[r4(orbit.target.x), r4(orbit.target.y)]}));

addEventListener('keydown', ev => {
  if(ev.target.closest && ev.target.closest('input,textarea')) return;
  const k = ev.key.toLowerCase(), mod = ev.ctrlKey || ev.metaKey, e = entity(selected);
  if(mod && k==='z' && !ev.shiftKey){ ev.preventDefault(); op('undo'); }
  else if(mod && (k==='y' || (k==='z' && ev.shiftKey))){ ev.preventDefault(); op('redo'); }
  else if(mod && k==='s'){ ev.preventDefault(); op('save'); }
  else if(mod && k==='d'){ ev.preventDefault(); if(e?.id) op('duplicate', {id:e.id}); }
  else if(mod) return;
  else if(k==='w') setMode('translate');
  else if(k==='e') setMode('rotate');
  else if(k==='f') focusSel();
  else if(k==='escape') select(null);
  else if((k==='delete' || k==='backspace') && e?.id) op('delete', {id:e.id});
});

fetch('/api/scene').then(r=>r.json()).then(sc=>applyScene(sc)).catch(()=>setStatus('Offline','err'));
</script>
</body></html>
"""

if __name__ == "__main__":
    main()
