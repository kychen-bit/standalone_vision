"""Mode-scoped colour parameters: the fill-light and ambient sets are separate.

The competition rules removed the venue's downward fill light, but the same
robot still has to run on the bench under a fill light while tuning. Those two
situations need different numbers:

* ``fill``    - a stable light the operator tuned against, white balance is not
                touched, the HSV ranges in ``config/jetson.json`` apply as-is.
* ``ambient`` - a downlight-free hall, every frame is white-balanced first, so
                the V axis means "fraction of the white field" and the ranges
                usually need to be wider.

Rather than duplicating the whole configuration, only the differences live in
``config/color_profiles.json``. The active section is deep-merged over
``colors`` / ``color_detection`` at start-up, so the operator can keep both sets
side by side and switch with ``--fill-light`` / ``--no-fill-light``.
"""

import json
from pathlib import Path
import time

PROFILE_FILE = "config/color_profiles.json"
# ``camera`` lets each lighting mode pick its own v4l2_control_profile, so
# switching fill_light also switches exposure/gain instead of relying on the
# operator to remember --camera-profile. A command line override still wins,
# because every entry point applies it after this merge.
PROFILE_SECTIONS = ("colors", "color_detection", "camera")


def _deep_merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def resolve_profile_name(config, override=None):
    """``fill`` / ``ambient`` / None (no switch configured)."""
    if override:
        return str(override).lower()
    fill_light = config.get("lighting", {}).get("fill_light")
    if fill_light is None:
        return None
    return "fill" if fill_light else "ambient"


def load_profile_file(project_root, path=None):
    profile_path = Path(path) if path else Path(project_root) / PROFILE_FILE
    if not profile_path.exists():
        return {}, profile_path
    return json.loads(profile_path.read_text(encoding="utf-8")), profile_path


def apply_color_profile(config, project_root, override=None, quiet=False):
    """Merge the active profile over the colour parameters in place.

    Idempotent: the override only ever adds or replaces keys, so calling this
    from both the engine builder and a tool is harmless. Keys starting with an
    underscore inside a ``colors`` block are documentation: merging them would
    put a string where the detector expects a colour definition.
    """
    name = resolve_profile_name(config, override)
    info = {
        "state": "COLOR_PROFILE",
        "profile": name,
        "file": None,
        "sections": [],
        "colors_overridden": [],
    }
    if name is None:
        if not quiet:
            print(json.dumps(info, ensure_ascii=False), flush=True)
        return info
    data, profile_path = load_profile_file(project_root)
    info["file"] = str(profile_path)
    profile = data.get(name) or {}
    for section in PROFILE_SECTIONS:
        block = profile.get(section)
        if not block:
            continue
        target = config.setdefault(section, {})
        if section == "colors":
            block = {
                key: value for key, value in block.items()
                if not str(key).startswith("_")
            }
            if not block:
                continue
            info["colors_overridden"] = sorted(block, key=str)
        _deep_merge(target, block)
        info["sections"].append(section)
    if not quiet:
        print(json.dumps(info, ensure_ascii=False), flush=True)
    return info


def save_override(project_root, name, section, payload, path=None):
    """Write ``payload`` into the profile file, keeping a backup."""
    if section not in PROFILE_SECTIONS:
        raise ValueError("unknown colour profile section: %s" % section)
    profile_path = Path(path) if path else Path(project_root) / PROFILE_FILE
    raw = None
    if profile_path.exists():
        raw = profile_path.read_text(encoding="utf-8")
        backup = profile_path.with_name(
            profile_path.name + "." + time.strftime("%Y%m%d-%H%M%S-%f") + ".bak"
        )
        backup.write_text(raw, encoding="utf-8")
    else:
        backup = None
    data = json.loads(raw) if raw else {}
    block = data.setdefault(name, {}).setdefault(section, {})
    _deep_merge(block, payload)
    temporary = profile_path.with_name(profile_path.name + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(profile_path)
    return str(backup) if backup else None
