#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate a headless-friendly living_room world with simple collision proxies.

The stock kaiaai living_room keeps every furniture *visual* mesh (those render
fine to the headless GPU-LiDAR), but Gazebo's dartsim engine builds no collision
for mesh shapes -> the robot drives straight through the sofa/table/chairs. This
adds a collision-only static primitive box for each mesh furniture item, sized to
the mesh's axis-aligned bounding box (trimesh handles the COLLADA up-axis and node
transforms; we then apply the SDF scale + link/model/world pose chain). Boxes carry
no visual, so the LiDAR still sees the real meshes; they exist purely so physics
stops the robot at the furniture.

Usage: gen_livingroom_proxies.py <stock_world> <models_dir> <out_world>
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

# decor / flat / on-wall / already-a-box models that need no floor collision proxy
SKIP_SUBSTR = ('poster', 'wall', 'ground', 'sun', 'curtain', 'rug', 'ball',
               'figurine', 'racoon', 'squirrel', 'door', 'cabinet', 'bookshelf',
               'tv_65', 'tv_stand_65', 'gate')
MAX_H = 0.9  # cap proxy height: nothing above this can be hit by the robot body


def pose_to_T(v):
    x, y, z, r, p, yw = (list(v) + [0] * 6)[:6]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yw), math.sin(yw)
    R = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
         @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
         @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def parse_pose(elem, default=(0, 0, 0, 0, 0, 0)):
    if elem is None:
        return np.array(default, float)
    p = elem.find('pose')
    if p is None or not p.text:
        return np.array(default, float)
    return np.array([float(t) for t in p.text.split()], float)


def mesh_bounds_scaled(models_dir, model_dir, model_sdf):
    """Return (bounds8x3 world-of-model-local corners) after scale, or None."""
    root = ET.parse(model_sdf).getroot()
    m = root.find('model')
    link = m.find('link')
    # first mesh geometry (visual or collision) with a uri
    mesh = None
    for g in m.iter('mesh'):
        if g.find('uri') is not None and g.find('uri').text:
            mesh = g
            break
    if mesh is None:
        return None
    uri = mesh.find('uri').text.strip()
    scl = mesh.find('scale')
    scale = np.array([float(t) for t in scl.text.split()], float) if (
        scl is not None and scl.text) else np.ones(3)
    if uri.startswith('model://'):
        rest = uri[len('model://'):]
        mesh_path = os.path.join(models_dir, rest)
    else:
        mesh_path = os.path.join(model_dir, uri)
    if not os.path.exists(mesh_path):
        print(f'  ! mesh not found: {mesh_path}', file=sys.stderr)
        return None
    g = trimesh.load(mesh_path, force='mesh')
    lo, hi = g.bounds  # local mesh AABB (up-axis + node xforms applied by trimesh)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    corners = corners * scale
    Pm = pose_to_T(parse_pose(m))
    Pl = pose_to_T(parse_pose(link))
    T = Pm @ Pl
    ch = np.c_[corners, np.ones(len(corners))] @ T.T
    return ch[:, :3]


def main():
    stock_world, models_dir, out_world = sys.argv[1:4]
    root = ET.parse(stock_world).getroot()
    world = root.find('world')
    proxies = []
    print(f'{"item":<20} {"center (x,y,z)":<26} {"size (x,y,z)":<22}')
    for model in world.findall('model'):
        name = model.get('name', '')
        low = name.lower()
        if any(s in low for s in SKIP_SUBSTR):
            continue
        inc = model.find('include')
        if inc is None or inc.find('uri') is None:
            continue
        model_uri = inc.find('uri').text.strip()
        mdir = os.path.join(models_dir, model_uri[len('model://'):])
        msdf = os.path.join(mdir, 'model.sdf')
        if not os.path.exists(msdf):
            continue
        local = mesh_bounds_scaled(models_dir, mdir, msdf)
        if local is None:
            continue  # box-primitive model (cabinet/bookshelf) -> already collides
        Tw = pose_to_T(parse_pose(model))
        cw = np.c_[local, np.ones(len(local))] @ Tw.T
        pts = cw[:, :3]
        lo, hi = pts.min(0), pts.max(0)
        zlo = max(0.0, lo[2])
        zhi = min(hi[2], MAX_H)
        if zhi <= zlo:
            zhi = min(max(hi[2], 0.2), MAX_H)
        sx, sy, sz = (hi[0] - lo[0]), (hi[1] - lo[1]), (zhi - zlo)
        cx, cy, cz = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (zlo + zhi) / 2
        print(f'{name:<20} ({cx:+.2f},{cy:+.2f},{cz:+.2f})       '
              f'({sx:.2f},{sy:.2f},{sz:.2f})')
        proxies.append(
            f'    <model name="{name}_col">\n'
            f'      <static>true</static>\n'
            f'      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <collision name="c">\n'
            f'          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}'
            f'</size></box></geometry>\n'
            f'        </collision>\n'
            f'      </link>\n'
            f'    </model>\n')

    # keep the stock world verbatim; drop physics to 200 Hz (lighter /clock);
    # inject the collision-only proxies just before </world>.
    text = open(stock_world).read()
    text = text.replace('<real_time_update_rate>1000</real_time_update_rate>',
                        '<real_time_update_rate>200</real_time_update_rate>')
    text = text.replace('<max_step_size>0.001</max_step_size>',
                        '<max_step_size>0.005</max_step_size>')
    block = ('\n    <!-- collision-only proxies: physics stops the robot at the\n'
             '         mesh furniture (dartsim builds no mesh collision). Visuals\n'
             '         stay stock so the LiDAR still sees the real meshes. -->\n'
             + ''.join(proxies))
    text = text.replace('</world>', block + '  </world>')
    open(out_world, 'w').write(text)
    print(f'\nwrote {out_world} with {len(proxies)} collision proxies', file=sys.stderr)


if __name__ == '__main__':
    main()
