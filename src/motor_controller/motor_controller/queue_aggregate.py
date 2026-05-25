"""
Optional NX-side action-chunk crossfade for joint_buf.

Linear blend over the overlap region when a new VLA chunk arrives before
the previous chunk drained:
    for i in range(overlap):
        w = i / overlap          # 0.0 → 1.0
        blended[i] = (1-w)*old[i] + w*new[i]
    blended[overlap:] = new[overlap:]

Canonical crossfade lives at the PC VLA Provider (next to inference).
This function is the NX-side fallback, activated when motor_controller
detects a chunk boundary (step_index==0 with a new chunk_id). OFF by
default.

Trap: zip() over (old_v, new_v) would silently truncate on length mismatch;
this function raises ValueError instead so a producer-side bug surfaces.

TODO(REQ-34, REQ-38) [TASK-34]: vectorise with numpy for jitter-free 20Hz.
"""
from typing import List


def crossfade(old_tail: List[List[float]],
              new_chunk: List[List[float]]) -> List[List[float]]:
    """Return the blended sequence to write into joint_buf.

    `old_tail` is the remaining joint vectors from the previous chunk
    (length = overlap). `new_chunk` (length >= overlap) is the incoming chunk.
    First `overlap` entries are linearly weighted; remainder copied from new.
    """
    overlap = len(old_tail)
    if overlap == 0:
        return list(new_chunk)
    if len(new_chunk) < overlap:
        raise ValueError(
            f'new_chunk shorter than overlap: {len(new_chunk)} < {overlap}')

    blended: List[List[float]] = []
    for i in range(overlap):
        w = i / overlap
        old_v = old_tail[i]
        new_v = new_chunk[i]
        if len(old_v) != len(new_v):
            raise ValueError(
                f'crossfade step {i}: joint-count mismatch '
                f'(old={len(old_v)} vs new={len(new_v)})')
        blended.append([(1.0 - w) * o + w * n for o, n in zip(old_v, new_v)])
    blended.extend(new_chunk[overlap:])
    return blended
