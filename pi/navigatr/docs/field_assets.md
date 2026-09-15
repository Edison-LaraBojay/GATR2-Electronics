# Field assets

Where the Override field numbers in `config/override/field.xml` come from,
how they were extracted, what they were checked against, and what is still
provisional. The CAD belongs to VEX Robotics; nothing from the STEP is
redistributed in this repository. The file ships numbers and their
provenance, never geometry.

## Source chain

| Item | Value |
|---|---|
| Permanent link | `https://link.vex.com/docs/26-27/v5rc/field-cad` |
| Redirects to | `https://content.vexrobotics.com/docs/2026-2027/override/files/v5rc-override-fieldcad.zip` |
| Zip size | 34,408,943 bytes |
| Zip ETag | `"69ef8cda-20d09ef"` |
| Zip Last-Modified | Mon, 27 Apr 2026 16:20:42 GMT |
| Zip content | one file, `276-9250-000 (2026-04-26).STEP`, 131,867,705 bytes |
| STEP header | `FILE_DESCRIPTION 'STEP AP214'`, `FILE_SCHEMA AUTOMOTIVE_DESIGN`, written 2026-04-26T17:34:18 by `SwSTEP 2.0` / SolidWorks 2024 |
| STEP units | millimeters (`SI_UNIT .MILLI. .METRE.`), Y up (SolidWorks convention) |
| Game manual | `https://link.vex.com/docs/26-27/v5rc/game-manual` -> `override-2.0.pdf`, Version 2.0 released 2026-09-03; Appendix A "Field Overview" sheets A1-A17 |

The server sends the same ETag for the only CAD revision seen; VEX could
replace the zip without a version note, so re-check the ETag before
trusting a local copy. The HEAD request is refused; a GET with a browser
user agent succeeds.

Manual history relevant to tags: v0.1 (2026-04-27) had no tag drawing;
v0.2 (2026-06-04) added sheet A16 "AprilTag numbering/locations" and the
toggle starting orientation; v1.0 (2026-07-02) added the metal-perimeter
toggle mount drawing (A17); v1.1 (2026-08-06) was a text reformat; v2.0
(2026-09-03) is current. The manual never states the tag family or the
printed size.

## Extraction method

- The STEP was parsed with a script (`stepgeo.py`, kept with the research
  notes, not in the repository) that walks the AP214 assembly tree,
  composes the `ITEM_DEFINED_TRANSFORMATION` placements, and reports the
  bounding box of each part's B-rep vertices. Curved features are
  represented by their control vertices only, so a bounding box can
  under-report by a fraction of a millimeter.
- Drawing dimensions were read from the manual PDF (Appendix A rendered at
  110 dpi). Where a drawing and the CAD differ, the manual number is used
  for display and the difference is noted.
- Nothing at runtime parses STEP, and no tessellated mesh ships with the
  binary. The viewer builds simple geometry (a box, an octagonal prism, a
  tape strip, a tag plate) from the declared dimensions in `field.xml`;
  see [inspection](inspection.md).

## CAD frame to field frame

The field frame (see [coordinates](coordinates.md)): origin at the inside
bottom-left corner of the perimeter on the tile surface, +x audience-right,
+y toward the 0-degree wall, +z up. The parser's conversion from the STEP
assembly frame (X right, Y up, Z toward the audience, origin at the field
center near the wall top):

```text
x = CAD_X + 1783.2
y = -CAD_Z + 1783.2
z = CAD_Y + 294.6
```

Verified against the manual: the planar part (`x`, `y`). Applying it to the
nine goal placements reproduces sheet A10's grid and sheet A16's id
assignment (red on the audience-left wall, blue on the right, id 0 at the
center), the loader placements reproduce A10's 290.6 / 3275.8 mm and 95 mm
projection, and the wall-to-wall distance reproduces A13. Two conventions
for the Z sign were possible; the other one put every toggle on one wall.

Parser's choice: the vertical offset. 294.6 mm is the distance from the
assembly origin to the parsed tile-top plane, and the goal sub-assemblies
did not all parse at that plane (the neutral-goal instances sat 64 mm
lower than the alliance goals in the same run). Every height used here is
therefore measured within its own goal assembly relative to the goal base
bottom and matched to sheet A7, never read off a global CAD Y. The toggle
sub-assemblies did not produce a clean bounding box either; the toggle
height is the manual's.

## Perimeter and tiles

| Item | Manual | CAD parse | field.xml |
|---|---|---|---|
| Inside wall to wall, portable 276-8242 (A13) | 140.40 in [3566.4 mm] | tiles 3608.4 mm overall (interlocking teeth) | `inside_x_m/inside_y_m="3.5664"` |
| Inside wall to wall, metal 278-1501 (A14) | 140.50 in [3568.7 mm] | not modeled | not used; A10/A12 lay out on 3566.4 |
| Wall height above tiles | 11.54 in [293 mm] (metal 292.1) | 293.0 mm | `wall_height_m="0.293"` |
| Wall thickness | 2.00 in [50.8 mm] (metal 32.4) | not read | `wall_thickness_m="0.0508"` |
| Tile pitch | 6 tiles per side | part 276-6904-001 is 618.1 mm square x 16 mm | `tile_m="0.5944"` = 3566.4 / 6 |

## Goal centers against the manual and the CAD

Sheet A10 grid lines: 23.11, 46.66, 70.20, 93.75, 117.30 in, printed as
587.1, 1185.1, 1783.2, 2381.3, 2979.3 mm. CAD centers are at +/-598.06 and
+/-1196.12 mm from the field center. `field.xml` values are in meters;
the difference column is field.xml minus the manual inches converted at
25.4 mm/in.

| Landmark | Manual (in) | Manual [mm] | CAD -> field (mm) | field.xml (m) | diff (mm) |
|---|---|---|---|---|---|
| neutral_goal_0_center | 70.20, 70.20 | 1783.2, 1783.2 | 1783.20, 1783.20 | 1.783200000, 1.783200000 | +0.12, +0.12 |
| neutral_goal_1_west | 23.11, 93.75 | 587.1, 2381.3 | 587.08, 2381.26 | 0.587082932, 2.381258534 | -0.01, +0.01 |
| neutral_goal_1_east | 117.30, 46.66 | 2979.3, 1185.1 | 2979.32, 1185.14 | 2.979317068, 1.185141466 | -0.10, -0.02 |
| neutral_goal_4_north | 46.66, 117.30 | 1185.1, 2979.3 | 1185.14, 2979.32 | 1.185141466, 2.979317068 | -0.02, -0.10 |
| neutral_goal_4_south | 93.75, 23.11 | 2381.3, 587.1 | 2381.26, 587.08 | 2.381258534, 0.587082932 | +0.01, -0.01 |
| red_goal_2_west | 23.11, 46.66 | 587.1, 1185.1 | 587.08, 1185.14 | 0.587082932, 1.185141466 | -0.01, -0.02 |
| red_goal_3_south | 46.66, 23.11 | 1185.1, 587.1 | 1185.14, 587.08 | 1.185141466, 0.587082932 | -0.02, -0.01 |
| blue_goal_2_east | 117.30, 93.75 | 2979.3, 2381.3 | 2979.32, 2381.26 | 2.979317068, 2.381258534 | -0.10, +0.01 |
| blue_goal_3_north | 93.75, 117.30 | 2381.3, 2979.3 | 2381.26, 2979.32 | 2.381258534, 2.979317068 | +0.01, -0.10 |

Every configured center is within 0.13 mm of the manual; the manual's own
element tolerance is +/-1 in. The tag ids per goal follow sheet A16: 0 at
the center, 1 on the west and east short neutral goals, 4 on the north and
south ones, 2 on the red west and blue east alliance goals, 3 on the red
south and blue north ones. Every id but 0 appears on two goals related by a
180 degree rotation about the center, so an id alone never identifies a
goal.

## Tag mount geometry, configured against CAD

CAD part 276-9250-006 ("AprilTag N") appears four times per goal, once on
each axis-aligned octagon face, none on the diagonal faces.

| Item | CAD-derived | Configured (`PoseOfTagSurfaceInLandmark`) |
|---|---|---|
| Plate outer dimensions | 56.0 wide x 50.8 tall x 4.9 mm thick | `Visual tag_plate_*` 0.056 x 0.0508 x 0.0049 |
| Plate outer face from goal center | 51.4 mm (the plate's outermost point) | `x_m` / `y_m` = 0.04863581744 (48.64 mm) |
| Plate center above tiles | 31.3 / 95.3 / 171.5 mm (51.2 mm below the goal top) | `z_m` = 0.0378522596 / 0.1018522596 / 0.1780522596 (37.85 / 101.85 / 178.05 mm) |
| Face orientation | relief face sloped about 5 degrees, top leaning toward the goal center | `pitch_deg="-5"`, `yaw_deg` = 0 / 90 / 180 / -90 |
| Relief pattern | 9 cells, 31.7 mm across, 3.51-3.52 mm per cell, spanning 22.06-53.64 mm above the plate bottom | detection size 0.01761272 m = 5/9 of 31.703 mm |

The two columns describe different points and both are consistent with
the STEP. The plate bounding box is a wedge: its outer face runs from
46.5 mm out at the top to 51.4 mm out at the bottom, and the printed
relief is not centered on the plate (the plate extends 16 mm further down
than the pattern). The configured origin is the center of the relief
pattern on that sloped face: 37.85 mm is the midpoint of the pattern's
22.06-53.64 mm vertical span, 48.64 mm is where the sloped face sits at
that height, and the -5 degree pitch is the face slope (1.8 mm of run over
21 mm of rise between the parsed layers, 4.9 degrees). That is the right
origin for the detector-corner frame, so the values were kept as found and
not replaced by the plate-box numbers. They stay `provisional` because
nobody has yet measured a physical goal; the parse of nested transforms
is the weakest link (see the vertical caveat above).

## Tag family and size

The manual does not name a family or a size. The inference:

- VEX's printable AprilTag sheet (`link.vex.com/apriltags-pdf`, March
  2024) states "AprilTag family = Circle21h7"; the VEX Library and the AIM
  API describe 38 ids (0-37), which is exactly the Circle21h7 code count.
- The CAD relief is a 9 x 9 cell pattern 31.7 mm across, matching
  Circle21h7's 9-cell layout.
- For tagCircle21h7 the detector's pose-estimation square is the
  black/white border, 5 of 9 cells: 31.7 x 5/9 = 17.61 mm. The configured
  `detection_size_m="0.01761272"` corresponds to a 31.703 mm printed side.

This is strongly inferred, not manual-stated. Measure the printed pattern
on a physical goal before trusting metric ranges, and confirm that a
tagCircle21h7 detector decodes ids 0-4 on the kit; the size in
`field.xml` (36 mounts) and in the detector resource must change
together.

## Display data

`Dimensions`, `Feature` and `Visual` elements are read only by the
inspection viewer; estimation never sees them.

- Loaders (A9, A10): boxes 95 x 102.1 x 583.5 mm against the red and blue
  walls at y = 290.6 and 3275.8 mm.
- Toggles (A8, A10): 660.2 mm boxes with a 52 mm section centered on
  every wall, top at 335.3 mm above the wall base as drawn on A8. The
  reference edge of that dimension (tile surface or floor under the
  tiles) is not legible at the rendered resolution.
- Midfield diamond (A11): four 864.8 x 63.5 mm tape strips at 45 degrees.
  A11 does not say whether 864.8 mm is the centerline or an edge; A12
  draws the vertices on the 598.06 mm goal grid, which implies an 845.8 mm
  centerline side. Kept as the printed number.
- Omitted: the two diagonal autonomous lines. A11 says they run corner to
  corner around the midfield, but nothing dimensions where they start or
  how they meet the diamond, so they are not drawn rather than guessed.
- Goals (A7): octagonal prisms, 142.5 mm across flats at the base, 88.8 mm
  at the top, 82.5 / 146.5 / 222.7 mm tall, with the 56 x 50.8 x 4.9 mm
  plate drawn behind every mount.

## Checks the test performs

`tests/field_assets_gtest.cpp`, against the checked-in file:

- Dimensions equal the manual (3.5664 m square, 0.293 m wall, 0.0508 m
  thickness, tile pitch inside/6); every landmark has a prism visual of
  its height class with the same plate size and one color per alliance.
- The nine nominal centers equal the A10 grid within 1 mm, with the A16 id
  on all four mounts of each goal.
- Symmetry: rotating any goal 180 degrees about the center lands on a goal
  with the same id; red goals lie on the small-x (and small-y) half, blue
  on the other. Scale and sign: every mount origin sits on one goal axis,
  between the top and base across-flats radii, its normal pointing out
  along that axis, at a height above the tiles and below the goal top;
  the four mounts of a goal cover +x, -x, +y, -y; 36 unique instance ids,
  all tagCircle21h7 at 0.01761272 m.
- Features parse and every rotated footprint stays inside the walls; the
  loaders, toggles and tape strips sit where the sheets put them.
- A Visual with an unknown shape and a Feature with a bad color fail with
  the attribute named.

## What remains provisional

- Physical check of goal centers on a real field (the manual allows
  +/-1 in).
- Physical check of the tag surface offset, height and slope on a real
  goal, and of the printed pattern size; then the family and size.
- Confirmation that the physical kit carries a tag on all four axis faces
  of every goal (the CAD does; the manual text never counts them).
- The toggle height reference edge and the diamond's reference edge
  (display only).
- Which perimeter the competition field uses (portable 3566.4 mm versus
  metal 3568.7 mm); the difference is 1.15 mm per half field.
