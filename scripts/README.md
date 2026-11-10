# Repository helper scripts

## Native drivers

- `windows/BUILD-DRIVER.cmd` — builds the current Windows native driver/runtime source.
- `linux/BUILD-DRIVER.sh` — builds the current Linux native driver/runtime source.
- `linux/INSTALL-DRIVER.sh` — installs the already-built Linux runtime offline; it never downloads or recompiles dependencies.

## SynKinect Studio

- `windows/BUILD-STUDIO.cmd` — bootstraps pinned Processing/JOGL/GlueGen dependencies, builds the Studio JAR, creates the bundled Java runtime and packages the native Windows application image.
- `windows/PACKAGE-APPLICATION.cmd` — packages the generated native Windows application image.
- `linux/BUILD-STUDIO.sh` — bootstraps pinned dependencies and JDK 17+ when needed, rebuilds the Studio JAR, and creates the self-contained Linux Java runtime with `jlink`.
- `windows/Build-Qt.ps1` and `linux/BUILD-QT.sh` — build the native Qt frontend alongside the Java Studio. Qt 6.4+ with Qt WebSockets and an ONNX Runtime CMake package are required; set `onnxruntime_DIR` if it is outside the standard CMake search path. The builders package the ONNX Runtime library and download the pinned MoveNet and RTMPose Hand models. Linux and Windows MP4 recording also needs FFmpeg on the application path.
- `macos/BUILD-STUDIO.sh` — packages the native Qt app bundle with ONNX Runtime and verified model assets.

Native driver outputs and generated Studio payloads are build outputs and are not repository inputs.

## Launcher path policy

Top-level and platform build entry points resolve paths relative to their own script location rather than the caller's working directory. Windows `.cmd` files remain ASCII/CRLF-safe; Linux `.sh` files are distributed executable and use script-root discovery that tolerates symlinks, spaces and wrapper directories introduced by repeated archive extraction.

The Studio builders install the SHA-256-pinned MoveNet RGB and RTMPose Hand ONNX models automatically. See their URLs and hashes in `config/build.env`. A failed model download or checksum check stops the build.
