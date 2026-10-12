# Lead correction 2026-09-28 15:05: readable plug-in folder names.
import re
R = 'C:/Users/mbes1/.claude/projects/c--GATR2-Electronics/restructure/'
FAMILY = {  # resources/resources/<file> -> resources/<family>/<file>
    'apriltag_c.c': 'tag_detectors', 'apriltag_c.h': 'tag_detectors',
    'tag_detectors.cpp': 'tag_detectors', 'tag_detectors.h': 'tag_detectors',
    'camera_capture.h': 'cameras', 'cameras.cpp': 'cameras', 'cameras.h': 'cameras',
    'libcamera_camera.cpp': 'cameras', 'libcamera_camera.h': 'cameras',
    'pico_telemetry.cpp': 'pico', 'pico_telemetry.h': 'pico',
    'pros_usb_link.cpp': 'links', 'pros_usb_link.h': 'links',
    'serial_links.cpp': 'links', 'serial_links.h': 'links',
    'field_map_resource.cpp': 'geometry', 'field_map_resource.h': 'geometry',
    'robot_frame_map.cpp': 'geometry', 'robot_frame_map.h': 'geometry',
    'target_set_resource.cpp': 'geometry', 'target_set_resource.h': 'geometry',
    'wheel_geometry_resource.cpp': 'geometry', 'wheel_geometry_resource.h': 'geometry',
    'synthetic_rig.cpp': 'synthetic', 'synthetic_rig.h': 'synthetic',
    'register_resources.cpp': None,  # stage level
}
SIMPLE = [
    ('pipeline/sensors/sensors/', 'pipeline/sensors/channels/'),
    ('pipeline/commands/command_collections/', 'pipeline/commands/collectors/'),
    ('pipeline/localization/observations/', 'pipeline/localization/observation_models/'),
    ('pipeline/targets/target_resolutions/', 'pipeline/targets/resolvers/'),
    ('pipeline/publishing/publishings/', 'pipeline/publishing/publishers/'),
]
def fix_path(p):
    m = re.match(r'(.*pipeline/resources/)resources/([^/]+)$', p)
    if m:
        fam = FAMILY[m.group(2)]
        return m.group(1) + (fam + '/' if fam else '') + m.group(2)
    for a, b in SIMPLE:
        p = p.replace(a, b)
    return p

tsv = R + 'plan/move_map_final.tsv'
lines = open(tsv, encoding='utf-8').read().splitlines()
out, changed = [], 0
for ln in lines:
    parts = ln.split('\t')
    if len(parts) >= 2:
        new = fix_path(parts[1])
        changed += new != parts[1]
        parts[1] = new
    out.append('\t'.join(parts))
open(tsv, 'w', encoding='utf-8', newline='\n').write('\n'.join(out) + '\n')
dests = [l.split('\t')[1] for l in out if '\t' in l and l.split('\t')[1] != 'same']
assert len(dests) == len(set(dests)), 'duplicate destination'
assert not any('/resources/resources/' in d or '/sensors/sensors/' in d or 'publishings' in d
               or 'command_collections' in d or 'target_resolutions' in d or '/observations/' in d for d in dests)
print('tsv destinations changed:', changed)

f = R + 'plan/FINAL.md'
s = open(f, encoding='utf-8').read()
for a, b in [('resources/resources/cameras.cpp', 'resources/cameras/cameras.cpp'),
             ('resources/resources/synthetic_rig.cpp', 'resources/synthetic/synthetic_rig.cpp'),
             ('resources/resources/pico_telemetry.h', 'resources/pico/pico_telemetry.h'),
             ('resources/resources/serial_links.h', 'resources/links/serial_links.h'),
             ('command_collections', 'collectors'), ('target_resolutions', 'resolvers'),
             ('publishings', 'publishers')]:
    s = s.replace(a, b)
s = s.replace('      resources/         <Resource> plug-ins, 26 files incl. register_resources.cpp',
              '      register_resources.cpp\n'
              '      cameras/           cameras, camera_capture.h, libcamera_camera\n'
              '      tag_detectors/     tag_detectors, apriltag_c\n'
              '      pico/              pico_telemetry\n'
              '      links/             pros_usb_link, serial_links\n'
              '      geometry/          field_map_resource, robot_frame_map, target_set_resource, wheel_geometry_resource\n'
              '      synthetic/         synthetic_rig')
s = s.replace('      sensors/           <Sensor> plug-ins, 7 files incl. register_sensors.cpp',
              '      channels/          <Sensor> plug-ins (attitude_channel, camera_channel, pico_channels), register_sensors.cpp')
s = s.replace('      observations/      attitude_reference', '      observation_models/ attitude_reference')
banner = ('# LEAD CORRECTION (2026-09-28 15:05), overrides everything below\n\n'
          'Plug-in folder names are readable plural nouns for what the plug-ins ARE, not a mechanical plural of the XML element. No stutter (resources/resources, sensors/sensors), no awkward plurals (publishings).\n'
          'Authoritative names:\n'
          '- resources: family folders directly under pipeline/resources/: cameras/, tag_detectors/, pico/, links/, geometry/, synthetic/; register_resources.cpp at stage level;\n'
          '- sensors: channels/;\n'
          '- commands: collectors/;\n'
          '- localization: observation_models/ and estimators/;\n'
          '- world_estimation: estimators/{apriltag,noop}/;\n'
          '- targets: resolvers/;\n'
          '- publishing: publishers/.\n'
          'The noop pieces follow the same folders (collectors/noop.*, resolvers/noop.*, publishers/noop.*, localization estimators/noop.*).\n'
          'move_map_final.tsv has been rewritten to these names by restructure/scripts/rename_plugins.py.\n'
          'Items C1 and O1 below are superseded. Any leftover old name in this file means the name above.\n\n')
s = banner + s
open(f, 'w', encoding='utf-8', newline='\n').write(s)
left = [w for w in ['resources/resources', 'sensors/sensors', 'command_collections', 'target_resolutions', 'publishings'] if w in s.split('# LEAD CORRECTION')[1].split('\n\n', 1)[1]]
print('FINAL.md leftovers after banner:', left)

sp = R + 'SPEC.md'
t = open(sp, encoding='utf-8').read()
old = "in a folder named after the concept that plugs in, matching its real XML element in plural form (e.g. <Estimator> gives estimators/). NEVER a folder named impl."
assert old in t
t = t.replace(old, "in a folder whose name is a readable plural noun for what the plug-ins are (estimators, collectors, resolvers, publishers, channels, observation_models); a stage with many plug-ins groups them by family instead (resources/cameras, resources/pico, ...). Never a mechanical plural of the XML element that reads badly, never a stutter (resources/resources), NEVER a folder named impl.")
open(sp, 'w', encoding='utf-8', newline='\n').write(t)
print('SPEC updated')
