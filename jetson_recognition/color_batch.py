"""Round-batch colour assignment helpers.

One preliminary round always carries three materials of three different
colours, so the observation is not "which colour is this blob" but "which
bijection between the visible blobs and this round's three colours explains the
measurements best". Independent per-blob decisions can label two blobs with the
same colour (blue and light blue overlap in hue, black and shadow overlap in
value); solving the whole assignment removes that.

This mirrors the inference the turntable strategy already uses for the raw
material positions.
"""

import itertools


def assign_distinct(scores, color_ids):
    """Assign one distinct colour per blob by maximising the total score.

    ``scores`` is a list with one mapping per blob of ``color_id -> score``,
    higher being a better match. Returns ``(assignment, total)`` where
    ``assignment`` is a list of colour IDs in blob order, or ``(None, 0.0)``
    when the counts do not match and no bijection exists.
    """
    ids = [str(value) for value in color_ids]
    rows = list(scores)
    if not rows or len(rows) != len(ids) or len(set(ids)) != len(ids):
        return None, 0.0
    best = None
    best_total = None
    for permutation in itertools.permutations(ids):
        total = 0.0
        for index, color_id in enumerate(permutation):
            total += float(rows[index].get(color_id, 0.0))
        if best_total is None or total > best_total + 1e-9:
            best_total = total
            best = list(permutation)
    return best, float(best_total)


def assignment_margin(scores, color_ids):
    """How much better the best bijection is than the runner-up."""
    ids = [str(value) for value in color_ids]
    rows = list(scores)
    if not rows or len(rows) != len(ids):
        return None
    totals = []
    for permutation in itertools.permutations(ids):
        totals.append(
            sum(float(rows[index].get(color_id, 0.0))
                for index, color_id in enumerate(permutation))
        )
    totals.sort(reverse=True)
    if len(totals) < 2:
        return float(totals[0]) if totals else None
    return float(totals[0] - totals[1])
