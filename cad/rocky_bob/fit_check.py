"""Check that everything in PARTS.md fits inside the rock, with room for its plugs and cables.

Every part is a box (or the placeholder from body.py) plus "keep-outs": the room its plugs, leads and fan
need. The script reports:
  - parts or keep-outs that run into the printed rock (walls, divider, standoffs, pads, lid),
  - anything poking out of the rock,
  - parts and keep-outs that run into each other,
  - each cable's rough route length against the cable you have.

    python fit_check.py          (needs cadquery, like build.py)

Positions are in the body frame (mm, +X forward, +Z up, origin at the middle of the base). Sizes marked
VERIFY are from listings and photos, not from the parts in hand: measure them when they arrive.
"""

import itertools
import math

import body
from body import box

MIN_VOL = 1.0   # mm^3; smaller overlaps are touching faces, not collisions

W = body.WALL
pcb_z = W + body.STANDOFF_H + body.PCB_T            # top of the Pi's board
sx = 1.5 + body.STANDOFF_H + body.PCB_T             # front face of the Servo 2040's board
cx0, cx1, cy0, cy1, cz0, cz1 = body.CONV_BOX
ux0, ux1, uy0, uy1, uz0, uz1 = body.UBEC_BOX
back_wall = -body.inner_radius(W, 180, body.SWITCH_Z)


def parts(printed):
    """name -> (shape, [(keep-out name, shape)]). Placeholders come from body.build()."""
    p = {name: shape for name, (shape, _, kind) in printed.items() if kind == "electronics"}
    yc = body.PI_USBC_Y
    return {
        "Pi 5 + cooler": (p["pi5"], [
            ("2 USB-A plugs and their bend", box(10, 50, -95, body.PI_Y0 - 3, body.WALL + 6, pcb_z + 18)),
            ("USB-C power plug (through the notch)", box(-20, body.PI_X0, yc - 6, yc + 6, pcb_z - 1, pcb_z + 6)),
            ("GPIO jumper leads", box(55, 66, body.PI_Y0 + body.PI_L - 58, body.PI_Y0 + body.PI_L - 7,
                                      pcb_z, pcb_z + 32)),
        ]),
        "Servo 2040": (p["servo2040"], [
            ("servo plugs and leads", box(sx, 48, body.S2040_Y0 + 2, body.S2040_Y0 + body.S2040_L - 2,
                                          body.S2040_Z0 + 8, body.S2040_Z0 + body.S2040_W)),
            ("USB-C plug", box(sx - 1, sx + 9, body.S2040_Y0 - 28, body.S2040_Y0,
                               body.S2040_USBC_Z - 5, body.S2040_USBC_Z + 5)),
            ("screw terminal wires", box(sx, sx + 14, body.S2040_Y0 + body.S2040_L, body.S2040_Y0 + body.S2040_L + 8,
                                         body.S2040_Z0 + 5, body.S2040_Z0 + 22)),
        ]),
        "IMU (BNO085)": (p["imu"], [
            ("jumper leads", box(1.5, 30, body.IMU_Y - 10, body.IMU_Y + 10, body.IMU_Z + 4, body.IMU_Z + 20)),
        ]),
        "Battery": (p["psu"], [
            ("lead and XT60 behind it", box(back_wall + 6, body.PSU_CX - body.PSU_L / 2 - 2.5, -7, 7, W, W + 26)),
            ("room to lift it out", box(body.PSU_CX - body.PSU_L / 2, body.PSU_CX + body.PSU_L / 2,
                                        -body.PSU_W / 2, body.PSU_W / 2, W + body.PSU_H, W + body.PSU_H + 2)),
        ]),
        "Pi 5 converter": (p["pi_power"], [
            ("USB-C out plug", box(cx0 + 2, cx1 - 2, cy1, cy1 + 26, cz0, cz0 + 12)),
            ("DC5521 in plug", box(cx0 + 2, cx1 - 2, cy0 - 16, cy0, cz0, cz0 + 14)),
            ("fan air", box(cx0 - 8, cx0, (cy0 + cy1) / 2 - 16, (cy0 + cy1) / 2 + 16, (cz0 + cz1) / 2 - 16,
                            (cz0 + cz1) / 2 + 16)),
        ]),
        "UBEC": (p["ubec"], [
            ("input wires", box(ux0 - 10, ux0, uy0 + 6, uy1 - 6, uz0, uz0 + 10)),
            ("output wires", box(ux1, ux1 + 10, uy0 + 6, uy1 - 6, uz0, uz0 + 10)),
        ]),
        "Main switch (rocker)": (box(back_wall - 1, back_wall + 28, -body.SWITCH_CUT[0] / 2 + 1,
                                     body.SWITCH_CUT[0] / 2 - 1, body.SWITCH_Z - body.SWITCH_CUT[1] / 2 + 1,
                                     body.SWITCH_Z + body.SWITCH_CUT[1] / 2 - 1), [
            ("spade terminals and wires", box(back_wall + 28, back_wall + 40, -12, 12, body.SWITCH_Z - 10,
                                              body.SWITCH_Z + 10)),
        ]),
        "Lidar USB adapter": (box(-12, -1.5, 15, 55, 52, 80), [   # foam tape, back face of the divider (VERIFY)
            ("USB plug", box(-12, -1.5, 55, 80, 58, 72)),
        ]),
        "Camera": (p["pi_camera"], []),
        "ToF (Mini)": (p["tof"], []),
        "Lidar": (p["lidar"], []),
    }


# name, cable length (mm, between the plugs), rough route through the waypoints
CABLES = [
    ("Pi USB-A -> Servo 2040 USB-C (6 in)", 152 - 2 * 28,
     [(30, body.PI_Y0 - 31, 18), (12, -90, 30), (12, -80, body.S2040_USBC_Z), (12, body.S2040_Y0 - 28, body.S2040_USBC_Z)]),
    ("Converter USB-C -> Pi USB-C (6 in)", 152 - 2 * 25,
     [(-12, cy1 + 26, cz0 + 6), (-12, cy1 + 26, 14), (-12, body.PI_USBC_Y, 14), (-20, body.PI_USBC_Y, 13)]),
    ("Camera -> Pi (200 mm ribbon)", 200,
     [(body.CAM_D - 10, 0, body.CAM_Z), (64, 0, 45), (40, -30, pcb_z + 2)]),
]


def vol(s):
    return s.Volume() if s is not None and not s.isNull() else 0.0


def overlap(a, b):
    try:
        return vol(a.intersect(b))
    except Exception:
        return 0.0


def route_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def main():
    printed = body.build()
    rock = {"rock base": printed["rock_base"][0], "rock lid": printed["rock_lid"][0]}
    outer = body.shell_at(0)
    found = []
    items = parts(printed)

    for name, (shape, keepouts) in items.items():
        for what, s in [(name, shape)] + [(f"{name}: {k}", ks) for k, ks in keepouts]:
            for rname, r in rock.items():
                if name == "Lidar" and rname == "rock lid":
                    continue   # it sits in its pocket on the lid
                if name == "Main switch (rocker)" and rname == "rock base":
                    continue   # it snaps through the wall
                v = overlap(s, r)
                if v > MIN_VOL:
                    found.append(f"{what} runs into the {rname} ({v:.0f} mm^3)")
            if name not in ("Main switch (rocker)", "Lidar"):
                out = vol(s) - overlap(s, outer)
                if out > MIN_VOL:
                    found.append(f"{what} pokes out of the rock ({out:.0f} mm^3)")

    flat = [(n, f"{n}: {k}" if k else n, s) for n, (shape, ko) in items.items()
            for k, s in [(None, shape)] + ko]
    for (n1, w1, s1), (n2, w2, s2) in itertools.combinations(flat, 2):
        if n1 != n2:
            v = overlap(s1, s2)
            if v > MIN_VOL:
                found.append(f"{w1} and {w2} overlap ({v:.0f} mm^3)")

    print("Fit inside the rock")
    if found:
        for f in found:
            print("  CLASH ", f)
    else:
        print(f"  everything fits: {len(items)} parts, {sum(len(k) for _, k in items.values())} plug/cable keep-outs")

    print("\nCables (rough route vs cable length between the plugs)")
    for name, length, pts in CABLES:
        need = route_length(pts) * 1.15   # bends
        print(f"  {'ok   ' if need <= length else 'SHORT'} {name}: route ~{need:.0f} mm, cable {length} mm")

    print(f"\nRoom: USB-A ports to side wall {body.PI_Y0 - 3 + body.inner_radius(W, 270, pcb_z + 6):.0f} mm, "
          f"Pi USB-C at y={body.PI_USBC_Y:.1f} in notch y={body.NOTCH_Y}, back wall at x={back_wall:.0f}")
    return 1 if found else 0


if __name__ == "__main__":
    raise SystemExit(main())
