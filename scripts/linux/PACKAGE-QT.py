#!/usr/bin/env python3
"""Build and archive the native Qt frontend and Kinect v2 runtime on Linux AMD64."""
import argparse
import hashlib
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
# Keep the host glibc and loader together. The release host sets the glibc floor.
SYSTEM = re.compile(r'^(ld-linux.*|lib(c|m|mvec|dl|pthread|rt|resolv|util|anl|BrokenLocale|nss_.*)\.so(?:\..*)?)$')
PLUGIN_DIRS = ('platforms', 'platforminputcontexts', 'imageformats', 'iconengines',
               'tls', 'multimedia', 'networkinformation', 'xcbglintegrations',
               'wayland-decoration-client', 'wayland-graphics-integration-client',
               'wayland-shell-integration')


def run(*args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)


def package(args):
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        raise SystemExit('Build this archive on Linux AMD64.')
    if not args.no_build:
        run('bash', str(ROOT / 'Kinect-Remold-linux-qt.sh'), '--build-only')
    source = ROOT / 'binaries/linux/applications/KinectRemoldQt'
    driver = ROOT / 'binaries/linux/drivers/kinect-one-remold/kinect-one-remold'
    qtpaths = args.qtpaths or shutil.which('qtpaths6')
    if not qtpaths:
        qt_bin = run('pkg-config', '--variable=bindir', 'Qt6Core', capture_output=True).stdout.strip()
        qtpaths = str(Path(qt_bin) / 'qtpaths')
    if not Path(qtpaths).is_file():
        raise SystemExit('Qt 6 qtpaths is required; select it with --qtpaths /path/to/qt6/bin/qtpaths.')
    if not run(qtpaths, '--qt-version', capture_output=True).stdout.strip().startswith('6.'):
        raise SystemExit('Selected qtpaths is not Qt 6. Use --qtpaths with the build SDK tool.')
    plugins = Path(run(qtpaths, '--plugin-dir', capture_output=True).stdout.strip())
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='remold-release-') as work:
        stage = Path(work) / 'KinectRemoldQt'
        binary, lib = stage / 'bin', stage / 'lib'
        binary.mkdir(parents=True)
        lib.mkdir()
        for name in ('models', 'licenses'):
            shutil.copytree(source / name, binary / name)
        for name, origin in [('KinectRemoldQt', source / 'KinectRemoldQt'),
                             ('kinect-one-remold', driver)]:
            shutil.copy2(origin, binary / name)
        shutil.copy2(ROOT / 'drivers/modules/kinect-one-remold/linux/source/70-kinect-one-remold.rules', binary)
        elfs = [binary / 'KinectRemoldQt', binary / 'kinect-one-remold']
        origins = set()
        if args.ffmpeg:
            encoder = shutil.which('ffmpeg')
            if not encoder:
                raise SystemExit('--ffmpeg requires ffmpeg on the build host.')
            shutil.copy2(encoder, binary / 'ffmpeg')
            origins.add(str(Path(encoder).resolve()))
            elfs.append(binary / 'ffmpeg')
        for name in PLUGIN_DIRS:
            if (plugins / name).is_dir():
                dest = stage / 'plugins' / name
                dest.mkdir(parents=True)
                for plugin in (plugins / name).glob('lib*.so'):
                    # Third-party desktop image plugins are unrelated to capture
                    # and can have optional, uninstalled codec dependencies.
                    if name == 'imageformats' and not plugin.name.startswith('libq'):
                        continue
                    if name == 'platforms' and plugin.name not in (
                            'libqxcb.so', 'libqwayland.so', 'libqoffscreen.so', 'libqminimal.so'):
                        continue
                    if name == 'platforminputcontexts' and plugin.name not in (
                            'libcomposeplatforminputcontextplugin.so', 'libibusplatforminputcontextplugin.so'):
                        continue
                    shutil.copy2(plugin, dest / plugin.name)
                    origins.add(str(plugin.resolve()))
        elfs.extend((stage / 'plugins').rglob('*.so'))
        copied = {}
        for elf in elfs:
            result = run('ldd', str(elf), capture_output=True).stdout
            if '=> not found' in result:
                raise RuntimeError(f'Missing dependency of {elf}:\n{result}')
            for soname, path in re.findall(r'^\s*(\S+) => (/\S+) \(', result, re.M):
                if SYSTEM.match(soname):
                    continue
                dest = lib / soname
                origin = Path(path).resolve()
                if soname in copied and copied[soname] != origin and dest.read_bytes() != origin.read_bytes():
                    raise RuntimeError(f'Conflicting libraries named {soname}')
                if not dest.exists():
                    shutil.copy2(origin, dest)
                copied[soname] = origin
                origins.add(str(origin))
        # Preserve distribution copyright/license material for the copied libraries.
        license_dir = stage / 'licenses'
        license_dir.mkdir()
        if shutil.which('pacman'):
            packages = set()
            for origin in origins:
                found = subprocess.run(['pacman', '-Qqo', origin], text=True, capture_output=True)
                packages.update(found.stdout.splitlines())
            for name in packages:
                location = Path('/usr/share/licenses') / name
                if location.is_dir():
                    shutil.copytree(location, license_dir / name, symlinks=False)
        (stage / 'DEPENDENCIES.txt').write_text('\n'.join(sorted(origins)) + '\n')
        (binary / 'qt.conf').write_text('[Paths]\nPrefix=..\nPlugins=plugins\n')
        launcher = stage / 'KinectRemoldQt.sh'
        launcher.write_text('''#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
export LD_LIBRARY_PATH="$ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export QT_PLUGIN_PATH="$ROOT/plugins"
export QT_QPA_PLATFORM_PLUGIN_PATH="$ROOT/plugins/platforms"
unset QT_QPA_PLATFORMTHEME
exec "$ROOT/bin/KinectRemoldQt" "$@"
''')
        launcher.chmod(0o755)
        shutil.copy2(ROOT / 'docs/PORTABLE-DEPLOYMENT.md', stage / 'README.md')
        shutil.copy2(ROOT / 'LICENSE', stage / 'LICENSE')
        # Fail before archiving if the selected Qt plugins or ONNX SDK don't
        # actually work after relocation. No sensor is needed for these checks.
        for flag in ('--check-body-model', '--check-hand-model', '--check-face-model'):
            run(str(launcher), flag, timeout=60, env={**os.environ,
                'QT_QPA_PLATFORM': 'offscreen', 'QT_FORCE_STDERR_LOGGING': '1'})
        archive = output / 'KinectRemoldQt-linux-amd64.tar.gz'
        with tarfile.open(archive, 'w:gz') as tar:
            tar.add(stage, arcname=stage.name)
        with archive.open('rb') as stream:
            checksum = hashlib.sha256()
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                checksum.update(chunk)
            digest = checksum.hexdigest()
        archive.with_suffix(archive.suffix + '.sha256').write_text(f'{digest}  {archive.name}\n')
        print(archive)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-build', action='store_true', help='Package existing build outputs')
    parser.add_argument('--ffmpeg', action='store_true', help='Include FFmpeg and its shared libraries for MP4 recording')
    parser.add_argument('--qtpaths', help='qtpaths executable from the Qt 6 SDK used to build the app')
    parser.add_argument('--output', default=str(ROOT / 'binaries/releases'))
    package(parser.parse_args())
