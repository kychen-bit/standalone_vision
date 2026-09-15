"""Pure turntable inspection and three-color inference logic."""


class TurntablePickStrategy:
    """Decide whether to pick, inspect the next slot, or use inference."""

    def __init__(self, target_color, round_colors):
        self.target = str(target_color)
        self.round_colors = tuple(str(value) for value in round_colors)
        if len(self.round_colors) != 3 or len(set(self.round_colors)) != 3:
            raise ValueError("round_colors must contain three distinct colors")
        if self.target not in self.round_colors:
            raise ValueError("target color is not present in this round")
        self.observed = []

    def observe(self, color_id):
        color_id = str(color_id)
        if color_id not in self.round_colors:
            raise ValueError("observed color is outside this round")
        if color_id in self.observed:
            raise ValueError("the same material color was observed twice")
        self.observed.append(color_id)
        position = len(self.observed)
        if color_id == self.target:
            return {
                "action": "PICK_CURRENT",
                "position": position,
                "color": color_id,
                "inferred": False,
            }
        if position == 1:
            return {
                "action": "MOVE_NEXT_AND_CLASSIFY",
                "position": position + 1,
                "observed": list(self.observed),
            }
        remaining = list(set(self.round_colors).difference(self.observed))
        if len(remaining) != 1:
            raise RuntimeError("cannot infer the remaining turntable material")
        return {
            "action": "RETURN_START_AND_WAIT_TARGET",
            "target": self.target,
            "inferred_color": remaining[0],
            "inferred": True,
        }
