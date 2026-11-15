# Blender facial capture importer

1. In Blender, open **Edit → Preferences → Add-ons → Install from Disk** and select `import_remold_face.py`.
2. Open **File → Import → Kinect Remold Face Capture** and choose a recorded `.rmclip` or `.jsonl` file.
3. In the file browser options, choose **Target Mesh**. It must already have ARKit named shape keys, such as `jawOpen` and `mouthSmileLeft`. The active mesh is preselected when possible.
4. Set **Start Frame** if needed, then import. The importer places shape-key animation at the scene frame rate. A multi-person clip can be filtered with **Track ID**.

Optional head motion: choose **Head Armature** and enter its **Head Bone** name. The first captured head rotation becomes the neutral orientation. Head motion is skipped if the fused clip switches source sensors, because those quaternions are in different camera coordinate systems. Shape-key animation still imports across switches.

The importer does not create a mesh, shape keys, or a full FBX rig. It applies the captured ARKit channels to the chosen existing mesh. Missing shape keys are ignored; the import succeeds if at least one channel matches. Blender can then export the animated target rig as FBX if the destination workflow needs FBX.
