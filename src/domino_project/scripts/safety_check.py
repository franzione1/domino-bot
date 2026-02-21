#!/usr/bin/env python3
"""
Basic safety check: parses a URDF file (if available) and reports missing joint limits.
"""
import xml.etree.ElementTree as ET
import os

urdf_candidates = [
    os.path.join(os.getcwd(), 'install', 'domino_project', 'share', 'domino_project', 'urdf', 'panda.urdf'),
    os.path.join(os.getcwd(), 'src', 'domino_project', 'urdf', 'simple_arm.urdf')
]

found = False
for p in urdf_candidates:
    if os.path.exists(p):
        found = True
        root = ET.parse(p).getroot()
        joints = root.findall('joint')
        for j in joints:
            limits = j.find('limit')
            if limits is None:
                print(f"WARNING: joint {j.get('name')} has no limit tag")
            else:
                if 'lower' not in limits.keys() or 'upper' not in limits.keys():
                    print(f"WARNING: joint {j.get('name')} missing lower/upper limits")
        print('Safety check completed for', p)
        break
if not found:
    print('No URDF found to perform safety checks.')
