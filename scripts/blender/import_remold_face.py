"""Import Kinect Remold facial capture onto an existing ARKit shape-key mesh.

Install in Blender via Edit > Preferences > Add-ons > Install from Disk, then use
File > Import > Kinect Remold Face Capture. The active mesh is the target.
"""

bl_info = {
    "name": "Kinect Remold Face Capture",
    "author": "Kinect Remold",
    "version": (1, 0, 0),
    "blender": (4, 0, 0),
    "location": "File > Import > Kinect Remold Face Capture",
    "description": "Keyframe existing ARKit shape keys from a Remold clip",
    "category": "Import-Export",
}

import json
import math
import struct
import zlib
import bpy
from bpy.props import EnumProperty, IntProperty, StringProperty
from bpy_extras.io_utils import ImportHelper
from mathutils import Matrix, Quaternion


def read_frames(path):
    """Yield JSON frames from JSONL or RMC1/QCompress clips."""
    with open(path, "rb") as stream:
        compact = stream.read(4) == b"RMC1"
        if not compact:
            stream.seek(0)
            for number, line in enumerate(stream, 1):
                if line.strip():
                    try:
                        yield json.loads(line)
                    except (ValueError, UnicodeDecodeError) as exc:
                        raise ValueError(f"Invalid JSON at line {number}: {exc}") from exc
            return
        number = 0
        while header := stream.read(4):
            number += 1
            if len(header) != 4:
                raise ValueError(f"Truncated frame {number} header")
            (size,) = struct.unpack("<I", header)
            if not 5 <= size <= 8 * 1024 * 1024:
                raise ValueError(f"Invalid frame {number} size: {size}")
            packed = stream.read(size)
            if len(packed) != size:
                raise ValueError(f"Truncated frame {number}")
            expected = struct.unpack(">I", packed[:4])[0]
            data = zlib.decompress(packed[4:])
            if len(data) != expected or len(data) > 8 * 1024 * 1024:
                raise ValueError(f"Invalid frame {number} payload")
            yield json.loads(data)


def face_from_frame(frame, track_id=""):
    tracks = frame.get("tracks") or []
    if track_id:
        for track in tracks:
            if track.get("id") == track_id or track.get("sensor") == track_id:
                return track.get("face") or {}
        return {}
    face = frame.get("face") or {}
    if face:
        return face
    for track in tracks:
        face = track.get("face") or {}
        if face.get("tracked"):
            return face
    return {}


def import_clip(path, mesh, scene, start_frame=1, track_id="", head_bone=None):
    if mesh.type != "MESH" or not mesh.data.shape_keys:
        raise ValueError("Select a mesh with existing ARKit shape keys")
    keys = mesh.data.shape_keys.key_blocks
    by_name = {key.name.casefold(): key for key in keys}
    first_time = None
    keyed = set()
    frame_count = 0
    face_count = 0
    head_count = 0
    source_sensors = set()
    recorded_names = set()
    head_samples = []
    last_frame = -1
    fps = scene.render.fps / scene.render.fps_base
    if fps <= 0:
        raise ValueError("Scene frame rate must be positive")
    for sample in read_frames(path):
        frame_count += 1
        face = face_from_frame(sample, track_id)
        if not face.get("tracked"):
            continue
        shapes = face.get("blendshapes") or {}
        if not shapes:
            continue
        recorded_names.update(shapes)
        timestamp = sample.get("t")
        if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError(f"Frame {frame_count} has no finite timestamp")
        if first_time is None:
            first_time = timestamp
        number = start_frame + round((timestamp - first_time) * fps)
        if number < start_frame:
            raise ValueError("Capture timestamps run backward")
        # A capture can run faster than the target scene. The last sample at
        # each scene frame wins, with no duplicate keyframe points.
        for name, value in shapes.items():
            key = by_name.get(name.casefold())
            if key is None or not isinstance(value, (int, float)) or not math.isfinite(value):
                continue
            key.value = max(0.0, min(1.0, float(value)))
            key.keyframe_insert(data_path="value", frame=number, group="Remold Face")
            keyed.add(key.name)
        source = face.get("sourceSensor") or sample.get("device") or ""
        source_sensors.add(source)
        q = (face.get("head") or {}).get("q")
        if head_bone and isinstance(q, list) and len(q) == 4 and all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in q):
            head_samples.append((number, source, Quaternion((q[3], q[0], q[1], q[2]))))
        face_count += 1
        last_frame = max(last_frame, number)
    if not face_count:
        raise ValueError("The clip contains no tracked face frames with blendshapes")
    if not keyed:
        raise ValueError("No recorded blendshape names match the selected mesh's shape keys")
    # The recorded head quaternion is in the source camera's space. It cannot
    # be baked continuously when fusion switches sensors without room alignment.
    if head_bone and len(source_sensors) == 1 and head_samples:
        camera_to_blender = Matrix(((1, 0, 0), (0, 0, 1), (0, -1, 0)))
        basis = camera_to_blender.to_quaternion()
        first = basis @ head_samples[0][2] @ basis.inverted()
        head_bone.rotation_mode = "QUATERNION"
        rest = head_bone.rotation_quaternion.copy()
        for number, _, camera_q in head_samples:
            rotation = basis @ camera_q @ basis.inverted()
            head_bone.rotation_quaternion = rest @ (first.inverted() @ rotation)
            head_bone.keyframe_insert(data_path="rotation_quaternion", frame=number,
                                      group="Remold Head")
            head_count += 1
    scene.frame_end = max(scene.frame_end, last_frame)
    scene.frame_set(start_frame)
    return {
        "capture_frames": frame_count,
        "face_frames": face_count,
        "shape_keys": len(keyed),
        "missing_keys": sorted(name for name in recorded_names if name.casefold() not in by_name),
        "head_frames": head_count,
        "head_skipped_source_switch": bool(head_bone and len(source_sensors) > 1),
    }


def mesh_choices(_self, _context):
    return [(obj.name, obj.name, "") for obj in bpy.data.objects if obj.type == "MESH"]


def armature_choices(_self, _context):
    return [("", "Auto", "Use the target mesh's armature")] + [
        (obj.name, obj.name, "") for obj in bpy.data.objects if obj.type == "ARMATURE"]


class IMPORT_SCENE_OT_remold_face(bpy.types.Operator, ImportHelper):
    bl_idname = "import_scene.remold_face"
    bl_label = "Import Kinect Remold Face Capture"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".rmclip"
    filter_glob: StringProperty(default="*.rmclip;*.jsonl", options={"HIDDEN"})
    target_mesh: EnumProperty(name="Target Mesh", items=mesh_choices)
    target_armature: EnumProperty(name="Head Armature", items=armature_choices)
    start_frame: IntProperty(name="Start Frame", default=1, min=0)
    track_id: StringProperty(name="Track ID", description="Optional person ID or sensor ID for multi-person clips")
    head_bone: StringProperty(name="Head Bone", description="Optional pose bone in the mesh's armature; only keyed for one source sensor")

    def invoke(self, context, event):
        if context.active_object:
            if context.active_object.type == "MESH":
                self.target_mesh = context.active_object.name
            elif context.active_object.type == "ARMATURE":
                self.target_armature = context.active_object.name
        return super().invoke(context, event)

    def execute(self, context):
        mesh = bpy.data.objects.get(self.target_mesh)
        if mesh is None:
            self.report({"ERROR"}, "Choose a target mesh with ARKit shape keys")
            return {"CANCELLED"}
        bone = None
        if self.head_bone:
            armature = bpy.data.objects.get(self.target_armature)
            if armature is None:
                armature = mesh.parent if mesh.parent and mesh.parent.type == "ARMATURE" else None
            if armature is None:
                armature = next((mod.object for mod in mesh.modifiers
                                 if mod.type == "ARMATURE" and mod.object), None)
            bone = armature.pose.bones.get(self.head_bone) if armature else None
            if bone is None:
                self.report({"ERROR"}, f"Head bone '{self.head_bone}' not found on the mesh's armature")
                return {"CANCELLED"}
        try:
            result = import_clip(self.filepath, mesh, context.scene, self.start_frame,
                                 self.track_id, bone)
        except (OSError, ValueError, zlib.error) as exc:
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        if result["head_skipped_source_switch"]:
            self.report({"WARNING"}, "Head bone skipped: fused clip switches camera space")
        if result["missing_keys"]:
            self.report({"WARNING"},
                        f"{len(result['missing_keys'])} captured channels have no target shape key")
        self.report({"INFO"}, f"Imported {result['face_frames']} face frames to "
                    f"{result['shape_keys']} shape keys")
        return {"FINISHED"}


def menu_func_import(self, _context):
    self.layout.operator(IMPORT_SCENE_OT_remold_face.bl_idname,
                         text="Kinect Remold Face Capture (.rmclip/.jsonl)")


def register():
    bpy.utils.register_class(IMPORT_SCENE_OT_remold_face)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    bpy.utils.unregister_class(IMPORT_SCENE_OT_remold_face)


if __name__ == "__main__":
    register()
