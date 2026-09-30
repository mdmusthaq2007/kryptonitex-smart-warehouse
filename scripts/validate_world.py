#!/usr/bin/env python3
import xml.etree.ElementTree as ET
from pathlib import Path

p = Path(__file__).resolve().parents[1] / 'worlds' / 'final_smart_warehouse_48x48x3.sdf'
root = ET.parse(p).getroot()
world = root.find('world')
models = world.findall('model') if world is not None else []
racks = [m for m in models if '_rack_' in m.get('name','')]
print(f'World: {world.get("name") if world is not None else "MISSING"}')
print(f'Model count: {len(models)}')
print(f'Rack model count: {len(racks)}')
print(f'File: {p}')
