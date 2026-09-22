# Choosing and splitting configuration

Run commands from `pi/navigatr` after building. On the Pi:

```sh
./build/navigatr
./build/navigatr --config_file="config/experiments/main.xml"
./build/navigatr --help
```

On Windows PowerShell with the repository's MSYS2 UCRT64 build:

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
.\build\navigatr.exe
.\build\navigatr.exe --config_file="config/examples/modular/main.xml"
```

The PATH entry supplies the matching MinGW runtime DLLs. Adjust it if MSYS2 is
installed elsewhere. Both `--config_file="path with spaces/main.xml"` and
`--config_file "path with spaces/main.xml"` work. `--config-file` is an alias;
a positional filename also works. Supplying multiple configuration filenames
is an error. A missing or invalid selected file fails startup; it does not
silently select another profile.

## Default file

With no filename, the executable reads the default selected at build time.
Initially this is [`config/demo/synthetic_field_demo.xml`](../config/demo/synthetic_field_demo.xml),
which runs the full synthetic robot, camera, detector, field estimator, and viewer.
Open `http://127.0.0.1:8765/` on the machine running it, or use the SSH tunnel
in [Inspection](inspection.md) when it runs on the Pi.

Choose a different default when configuring your build:

```sh
cmake -S . -B build "-DNAVIGATR_DEFAULT_CONFIG=config/experiments/main.xml"
cmake --build build -j4
./build/navigatr
```

CMake remembers this choice in that build directory. A relative default path is
resolved against the `pi/navigatr` source directory and compiled as an absolute
path, so launching from another working directory still finds it. An explicit
CLI filename is resolved from the shell's working directory.

The **path** is compiled in; the XML files remain external and are read at each
startup. Editing their values requires a restart, not a rebuild. Choosing a
different compiled default requires rebuilding. If you move the configuration
tree, provide `--config_file` or rebuild with its new location. `--help` and the
startup log show which file is selected.

## Runnable starter files

- The default synthetic demo provides moving localization, rendered tag images,
  landmark estimates, and the browser viewer without hardware.
- [`config/examples/modular/main.xml`](../config/examples/modular/main.xml) is a
  minimal editable scaffold with separate resource, sensor, pipeline, and
  localization XML files. Its no-op stages produce no robot estimate or reports.
- The `.xml.in` profiles under `config/override/diagnostics/` and descriptions
  under `config/shared/robots/` are the real-robot templates. Replace their
  measurement placeholders and save runnable `.xml` files before using them.

For example, exercise the scaffold for ten cycles:

```sh
./build/navigatr --config_file="config/examples/modular/main.xml" --inline --cycles 10
```

## Inline sections and file references

A main `<System>` can reference its sections:

```xml
<System>
    <Loop rate_hz="100"/>
    <Resources file="./resources/resource_config1.xml"/>
    <Sensors file="./sensors/sensor_config1.xml"/>
    <Pipeline file="./pipelines/pipeline1.xml"/>
</System>
```

The resource file contains the same root you would write inline:

```xml
<Resources>
    <Resource id="scratch_link" type="memory_link"/>
</Resources>
```

Replacing the `Resources file="..."` element with that whole `<Resources>` block
has the same configuration meaning. Other sections can remain file references;
inline and referenced sections can be mixed.

The pattern also applies within pipelines:

```xml
<Pipeline>
    <CommandCollection type="noop"/>
    <Localization file="./localization.xml"/>
    <WorldEstimation file="./world_estimation.xml"/>
    <TargetResolution type="noop"/>
    <Publishing type="noop"/>
</Pipeline>
```

`localization.xml` has a `<Localization>` root, and `world_estimation.xml`
has a `<WorldEstimation>` root holding its one `<Estimator>`. That Estimator
can itself be a reference to a file whose root is `<Estimator>` with its `id`,
`type`, and implementation-owned children. Individual resource/sensor
declarations and localization observation/estimator nodes can also be extracted
to files at their respective configuration slots; the children of any of these
implementation nodes are opaque to the loader.

Paths resolve **relative to the XML file containing the reference**, at every
level. Absolute paths work too. Referenced roots must match their slots. A
reference is a replacement, so it cannot also contain inline configuration or
override attributes such as `type`; put those in the referenced root. Missing
files, cycles, repeated includes, conflicting declarations, and wrong roots are
configuration errors. Contributing files are included in the configuration
digest shown by the program.

File references apply at configuration sections and pipeline slots. A resource's
implementation-specific `<Device file="capture.bin"/>`, for example, remains a
device option; the loader does not try to parse that data file as XML.

Existing `<Configuration>` profiles with `Robot`, `Field`, and `Pipeline`
composition also support inline or referenced sections. A `Robot` groups
resources and sensors; `Field` accepts its existing single `<Resource>` or
`<Field>` wrapper. Both main-document forms resolve into the same runtime
`<System>` before factories run.

`calibration_status` remains a reader-only annotation, including on a reference.
It never enables a feature, approves a calibration, or suppresses numeric
validation.
