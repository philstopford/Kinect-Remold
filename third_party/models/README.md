# Face blendshape ONNX model

`face-blendshapes.onnx` was converted from `face_blendshapes.tflite` inside
Google's MediaPipe Face Landmarker task bundle, pinned in `config/build.env`.
The model and original graph are Apache-2.0 licensed, copyright Google LLC.
The conversion changes the file format and does not change model weights.

- Task bundle SHA-256: `64184e229b263107bc2b804c6625db1341ff2bb731874b0bcc2fe6544e0bc9ff`
- Extracted TFLite SHA-256: `4f36dded049db18d76048567439b2a7f58f1daabc00d78bfe8f3ad396a2d2082`
- ONNX SHA-256: `e76eecea12e7af3b5c41e8781a4260b509902f8971e20409af26f03281b9f822`
- Conversion: `tf2onnx 1.17.0`, TensorFlow `2.20.0`, ONNX opset 17.
- Input: `float32 [1,146,2]`, selected landmark X/Y coordinates in image pixels.
- Output: 52 MediaPipe coefficients, including `_neutral`. The model does not
  infer ARKit `tongueOut`.

The 146 landmark indices and coefficient names follow MediaPipe's
`face_blendshapes_graph.cc` from the Apache-2.0 MediaPipe repository.
