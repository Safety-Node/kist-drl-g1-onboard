"""
VLA action-chunk crossfade for joint_buf.

When a new action chunk arrives while the previous chunk still has 'overlap' actions
remaining, blend them linearly so the joint trajectory stays smooth.

Formula (per spec):
    for i in range(overlap):
        w = i / overlap                       # 0.0 → 1.0
        blended[i] = (1 - w) * old[i] + w * new[i]
    blended[overlap:] = new[overlap:]

Applies to joint_buf only. velocity_buf is unaffected (LocoClient handles walking inertia).

Embedded analogy: identical in spirit to S-curve velocity blending or audio crossfading.
"""
from typing import List


def crossfade(old_tail: List[List[float]],
              new_chunk: List[List[float]]) -> List[List[float]]:
    """
    Return the blended sequence to write into joint_buf.

    `old_tail` is the remaining joint vectors from the previous chunk
    (length = overlap). `new_chunk` is the incoming chunk
    (length >= overlap). The first len(old_tail) entries of the result
    are linearly weighted; the remainder is copied from `new_chunk`
    unchanged.
    """
    overlap = len(old_tail)
    if overlap == 0:
        return list(new_chunk)
    if len(new_chunk) < overlap:
        raise ValueError(
            f'new_chunk shorter than overlap: {len(new_chunk)} < {overlap}')

    # TODO(REQ-34, REQ-38): vectorise with numpy for jitter-free 20Hz operation.
    blended: List[List[float]] = []
    for i in range(overlap):
        w = i / overlap
        old_v = old_tail[i]
        new_v = new_chunk[i]
        blended.append([(1.0 - w) * o + w * n for o, n in zip(old_v, new_v)])
    blended.extend(new_chunk[overlap:])
    return blended
