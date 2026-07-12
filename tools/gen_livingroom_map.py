#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a world-aligned occupancy map for the living_room world.

The stock kaiaai living_room map was built by SLAM in a frame offset from the gz
world origin (and its free space leaks outside the walls), so it can't be used
with world-frame spawn / ground truth. This rasterizes every *box* collision in
the world -- walls, cabinet, bookshelf, and the furniture collision proxies --
directly in the world frame, then flood-fills the interior for free space. Result:
map frame == world frame == ground-truth frame, complete and repeatable.

Usage: gen_livingroom_map.py <world> <models_dir> <out_dir>
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy import ndimage

RES = 0.05
OX, OY = -2.75, -2.75      # world-aligned origin, room is ~[-2.5,2.5]^2
W = H = 110                # 5.5 x 5.5 m

SKIP = ('poster', 'sun', 'ground', 'curtain', 'rug', 'ball', 'figurine',
        'racoon', 'squirrel', 'tv_65', 'gate', 'tv_stand_65')
# the mesh furniture is represented by its *_col proxy box; skip the mesh model
MESH = ('sofa', 'minisofa', 'chair', 'tvstand', 'lampandstand',
        'femalevisitor', 'malevisitor', 'tablemarble', 'coffeetable')


def T_of(p):
    x, y, z, r, pi, yw = (list(p) + [0] * 6)[:6]
    cy, sy = math.cos(yw), math.sin(yw)          # roll/pitch are 0 in this world
    T = np.eye(4)
    T[0, 0], T[0, 1] = cy, -sy
    T[1, 0], T[1, 1] = sy, cy
    T[:3, 3] = [x, y, z]
    return T


def pose_of(elem):
    if elem is None:
        return [0, 0, 0, 0, 0, 0]
    p = elem.find('pose')
    if p is None or not p.text:
        return [0, 0, 0, 0, 0, 0]
    return [float(t) for t in p.text.split()]


def boxes_from_model_elem(model_elem, T_base):
    """World boxes (cx,cy,yaw,sx,sy) from a model element's box collisions."""
    out = []
    Pm = T_of(pose_of(model_elem))
    for link in model_elem.findall('link'):
        Pl = T_of(pose_of(link))
        for col in link.findall('collision'):
            size = col.find('.//box/size')
            if size is None or not size.text:
                continue
            sx, sy, _ = [float(t) for t in size.text.split()]
            Pc = T_of(pose_of(col))
            T = T_base @ Pm @ Pl @ Pc
            out.append((T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0]), sx, sy))
    return out


def main():
    world_path, models_dir, out_dir = sys.argv[1:4]
    world = ET.parse(world_path).getroot().find('world')
    boxes = []
    for model in world.findall('model'):
        name = model.get('name', '').lower()
        if any(s in name for s in SKIP):
            continue
        if any(s in name for s in MESH) and not name.endswith('_col'):
            continue  # mesh visual model; its proxy box carries the collision
        Tw = T_of(pose_of(model))
        inc = model.find('include')
        if inc is not None and inc.find('uri') is not None:
            mdir = os.path.join(models_dir, inc.find('uri').text.strip()[8:])
            msdf = os.path.join(mdir, 'model.sdf')
            if os.path.exists(msdf):
                me = ET.parse(msdf).getroot().find('model')
                boxes += boxes_from_model_elem(me, Tw)
        else:
            boxes += boxes_from_model_elem(model, np.eye(4))  # inline: pose in Tw
            # inline model pose is its own <pose>; fold via T_base=identity since
            # boxes_from_model_elem applies Pm=model pose itself
    occ = np.zeros((H, W), bool)
    gx, gy = np.meshgrid(np.arange(W), np.arange(H))
    wx = OX + (gx + 0.5) * RES
    wy = OY + (gy + 0.5) * RES
    for (cx, cy, yaw, sx, sy) in boxes:
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = c * (wx - cx) - s * (wy - cy)
        ly = s * (wx - cx) + c * (wy - cy)
        occ |= (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)

    # free = the room interior = the largest connected non-occupied region.
    # (seeding a fixed point is fragile: world (0,0) sits inside the table proxy.)
    lbl, n = ndimage.label(~occ)
    if n == 0:
        raise SystemExit('no free space found')
    sizes = ndimage.sum(np.ones_like(lbl), lbl, index=range(1, n + 1))
    room = 1 + int(np.argmax(sizes))
    free = (lbl == room)

    img = np.full((H, W), 205, np.uint8)
    img[free] = 254
    img[occ] = 0
    os.makedirs(out_dir, exist_ok=True)
    Image.fromarray(np.flipud(img), 'L').save(os.path.join(out_dir, 'living_room.pgm'))
    with open(os.path.join(out_dir, 'living_room.yaml'), 'w') as f:
        f.write(f"image: living_room.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{OX}, {OY}, 0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f'boxes={len(boxes)} free={int(free.sum())} occ={int(occ.sum())} '
          f'origin=({OX},{OY}) {W}x{H}')


if __name__ == '__main__':
    main()
