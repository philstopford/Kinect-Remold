"""Run with: blender -b --factory-startup --python check_import_remold_face.py"""

import importlib.util
import json
import struct
import sys
import tempfile
import zlib
from pathlib import Path

import bpy

sys.dont_write_bytecode = True
path = Path(__file__).with_name("import_remold_face.py")
spec = importlib.util.spec_from_file_location("remold_face", path)
addon = importlib.util.module_from_spec(spec)
spec.loader.exec_module(addon)
addon.register()


def mesh(name):
    geometry = bpy.data.meshes.new(name)
    geometry.from_pydata([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [], [(0, 1, 2)])
    obj = bpy.data.objects.new(name, geometry)
    bpy.context.collection.objects.link(obj)
    obj.shape_key_add(name="Basis")
    obj.shape_key_add(name="jawOpen")
    return obj


target = mesh("FaceTarget")
other = mesh("OtherMesh")
bpy.context.view_layer.objects.active = other
frames = [
    {"t": 10.0, "device": "a", "face": {"tracked": True, "sourceSensor": "a",
                                     "blendshapes": {"jawOpen": 0.2}}},
    {"t": 10.1, "device": "a", "face": {"tracked": True, "sourceSensor": "b",
                                     "blendshapes": {"jawOpen": 0.8}}},
]
with tempfile.TemporaryDirectory() as directory:
    jsonl = Path(directory) / "capture.jsonl"
    jsonl.write_text("".join(json.dumps(frame) + "\n" for frame in frames))
    compact = Path(directory) / "capture.rmclip"
    with compact.open("wb") as stream:
        stream.write(b"RMC1")
        for frame in frames:
            data = json.dumps(frame).encode()
            packed = struct.pack(">I", len(data)) + zlib.compress(data)
            stream.write(struct.pack("<I", len(packed)))
            stream.write(packed)
    assert list(addon.read_frames(compact)) == frames
    assert bpy.ops.import_scene.remold_face(
        filepath=str(jsonl), target_mesh=target.name, start_frame=3) == {"FINISHED"}
    assert target.data.shape_keys.animation_data.action is not None
    bpy.context.scene.frame_set(3)
    assert abs(target.data.shape_keys.key_blocks["jawOpen"].value - 0.2) < 1e-5
    bpy.context.scene.frame_set(5)
    assert abs(target.data.shape_keys.key_blocks["jawOpen"].value - 0.8) < 1e-5
    assert other.data.shape_keys.animation_data is None
    assert addon.import_clip(str(compact), target, bpy.context.scene, start_frame=10)["face_frames"] == 2

addon.unregister()
print("REMOLD_BLENDER_IMPORT_OK")
