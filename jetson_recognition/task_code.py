"""Validation and decoding for the initial-round competition task code.

This is the standalone_vision copy of ``new/common/task_code.py`` so the
Jetson coordinator does not depend on the navigation project. The MaixCAM Pro
board validates the QR payload with its own ``is_valid_task_code``; this module
turns a validated payload into the two-batch pickup plan used by the
coordinator.
"""

COLOR_NAMES = {
    "1": "RED",
    "2": "YELLOW",
    "3": "BLUE",
    "4": "GREEN",
    "5": "BLACK",
    "6": "LIGHT_BLUE",
}


def _is_group(text, allowed):
    return len(text) == 3 and all(character in allowed for character in text)


def is_valid_task_code(payload):
    groups = str(payload).strip().split("+")
    if len(groups) != 4:
        return False
    first_colors, first_positions, second_colors, second_positions = groups
    return (
        _is_group(first_colors, "123456")
        and _is_group(second_colors, "123456")
        and _is_group(first_positions, "123")
        and _is_group(second_positions, "123")
        and len(set(first_colors)) == 3
        and len(set(second_colors)) == 3
        and set(first_positions) == set("123")
        and set(second_positions) == set("123")
    )


def decode_task_code(payload):
    payload = str(payload).strip()
    if not is_valid_task_code(payload):
        raise ValueError("invalid competition task code")
    first_colors, first_positions, second_colors, second_positions = payload.split("+")
    batches = []
    for colors, positions in (
        (first_colors, first_positions),
        (second_colors, second_positions),
    ):
        batches.append(
            [
                {
                    "color_id": color,
                    "color_name": COLOR_NAMES[color],
                    "ring_id": position,
                }
                for color, position in zip(colors, positions)
            ]
        )
    return {"raw": payload, "batches": batches}


def flat_color_plan(payload):
    """Return the ordered color IDs of both batches, e.g. ['1','5','6','5','1','6']."""
    plan = decode_task_code(payload)
    return [
        item["color_id"] for batch in plan["batches"] for item in batch
    ]
