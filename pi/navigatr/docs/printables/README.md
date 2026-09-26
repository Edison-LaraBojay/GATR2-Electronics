# Printable calibration and coordinate guides

- [US Letter checkerboard](checkerboard-9x6-25mm-letter.pdf)
- [A4 checkerboard](checkerboard-9x6-25mm-a4.pdf)
- [Field coordinate guide](field-coordinate-guide.pdf)

The checkerboards have **10 columns by 7 rows of squares**, producing **9 by 6
inner corners**. Each square is drawn at **25 mm**. Both paper versions contain
the same target geometry. SVG copies are included as editable vector sources;
print the PDF matching your paper size.

Print in landscape at **Actual size / 100%**. Disable Fit, Shrink and borderless
expansion. Measure several squares in both directions after printing; use the
measured edge length in the calibration command. Keep the white margin and mount
the sheet flat on a rigid surface.

For a print that measures 25 mm per square, the board arguments for the existing
camera calibration tool are:

```text
--cols 9 --rows 6 --square-size-m 0.025
```

Follow the [camera calibration workflow](../setup.md) for capture and fitting.
Use the operating camera resolution, crop and fixed focus, and collect 20-30
sharp views at varied positions, distances and tilts. The resulting calibration
describes camera intrinsics; the camera mounting relative to the robot is
configured separately.

The coordinate guide follows [coordinates.md](../coordinates.md) and the current
[field definition](../../config/override/field.xml). The field origin is the
inside bottom-left corner in the audience-view drawing; +x is right, +y is up
the drawing, and heading increases counterclockwise from +x. Changing starting
sides changes the robot's field pose, not these axes. Robot drawings in the guide
illustrate headings and are not measured starting placements.
