#!/usr/bin/env python3
"""
package_slicer_app_macos.py

Generates a completely self-contained, standalone macOS application bundle
(SlicerROS2.app) and an installable disk image (.dmg) for Apple Silicon (arm64).

The generated application bundles:
  1. Native 3D Slicer application (arm64)
  2. SlicerROS2 loadable module (C++ Qt module, MRML, Logic, Python wrappers)
  3. minimal_ros2 C/C++ runtime libraries (librcl, librclcpp, CycloneDDS, etc.)
  4. Slicer Python runtime with rclpy and all standard ROS 2 message packages
  5. Required runtime Python dependencies (empy, lark, pyyaml, catkin_pkg, etc.)
  6. Homebrew Qt plugins (Cocoa QPA, OpenSSL/SecureTransport TLS backends, SQLite driver)
  7. Complete Mach-O relocation (@rpath/@loader_path) and ad-hoc code-signing

Usage:
  python3 scripts/package_slicer_app_macos.py
  python3 scripts/package_slicer_app_macos.py --slicer-app /path/to/Slicer.app
  python3 scripts/package_slicer_app_macos.py --no-dmg
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path


def is_macho(filepath):
    """Check if the file is a Mach-O binary."""
    if os.path.islink(filepath) or not os.path.isfile(filepath):
        return False
    try:
        with open(filepath, "rb") as f:
            magic = f.read(4)
            return magic in [
                b"\xfe\xed\xfa\xce",
                b"\xce\xfa\xed\xfe",
                b"\xfe\xed\xfa\xcf",
                b"\xcf\xfa\xed\xfe",
                b"\xca\xfe\xba\xbe",
                b"\xbe\xba\xfe\xca",
            ]
    except Exception:
        return False


def copy_file_or_symlink(src, dst_dir):
    """Copy a file or symlink into dst_dir, preserving symlink relations."""
    os.makedirs(dst_dir, exist_ok=True)
    basename = os.path.basename(src)
    dst_path = os.path.join(dst_dir, basename)
    if os.path.islink(src):
        link_target = os.readlink(src)
        if os.path.exists(dst_path) or os.path.islink(dst_path):
            os.remove(dst_path)
        os.symlink(link_target, dst_path)
    else:
        if os.path.exists(dst_path) or os.path.islink(dst_path):
            os.remove(dst_path)
        shutil.copy2(src, dst_path)
    return dst_path


def copy_shared_lib_with_symlinks(src_path, dst_dirs, alias_names=None):
    """
    Copies the real binary of src_path to each dir in dst_dirs with permissions,
    and creates symlinks for all version aliases (e.g. libfmt.12.dylib, libfmt.dylib).
    Returns list of all created real file paths.
    """
    real_src = os.path.realpath(src_path)
    if not os.path.exists(real_src):
        print(f"  Warning: real source not found for {src_path}")
        return []

    real_base = os.path.basename(real_src)
    src_base = os.path.basename(src_path)

    aliases = set()
    if alias_names:
        aliases.update(alias_names)
    aliases.add(src_base)

    created_real_paths = []
    for dst_dir in dst_dirs:
        os.makedirs(dst_dir, exist_ok=True)
        dst_real = os.path.join(dst_dir, real_base)
        if os.path.exists(dst_real) or os.path.islink(dst_real):
            os.remove(dst_real)
        shutil.copy2(real_src, dst_real)
        os.chmod(dst_real, 0o755)
        created_real_paths.append(dst_real)

        for alias in aliases:
            if alias != real_base:
                alias_path = os.path.join(dst_dir, alias)
                if os.path.exists(alias_path) or os.path.islink(alias_path):
                    os.remove(alias_path)
                os.symlink(real_base, alias_path)

    return created_real_paths


def copy_tree_preserving_symlinks(src_dir, dst_dir):
    """Copy directory tree preserving symlinks, returning list of all copied files."""
    copied = []
    os.makedirs(dst_dir, exist_ok=True)
    for root, dirs, files in os.walk(src_dir):
        rel = os.path.relpath(root, src_dir)
        target_root = os.path.join(dst_dir, rel) if rel != "." else dst_dir
        os.makedirs(target_root, exist_ok=True)
        for d in dirs:
            os.makedirs(os.path.join(target_root, d), exist_ok=True)
        for f in files:
            src_file = os.path.join(root, f)
            p = copy_file_or_symlink(src_file, target_root)
            copied.append(p)
    return copied


def find_base_slicer_app(slicer_app_arg, slicer_dir_arg, superbuild_dir):
    """Locate or extract the base standalone Slicer.app."""
    if slicer_app_arg and os.path.isdir(slicer_app_arg):
        print(f"Using specified Slicer.app: {slicer_app_arg}")
        return slicer_app_arg

    # Check _CPack_Packages in Slicer-build
    cpack_pattern = os.path.join(
        slicer_dir_arg or "/opt/slicer/Slicer-SuperBuild/Slicer-build",
        "_CPack_Packages",
        "*",
        "TGZ",
        "*",
        "*.app",
    )
    candidates = glob.glob(cpack_pattern)
    if candidates:
        print(f"Found CPack staged Slicer.app: {candidates[0]}")
        return candidates[0]

    # Check for Slicer-*.tar.gz package in Slicer-build
    tgz_pattern = os.path.join(
        slicer_dir_arg or "/opt/slicer/Slicer-SuperBuild/Slicer-build",
        "Slicer-*.tar.gz",
    )
    tgzs = glob.glob(tgz_pattern)
    if tgzs:
        tar_path = tgzs[0]
        print(f"Found Slicer tarball: {tar_path}. Extracting base app...")
        extract_dir = os.path.join(os.path.dirname(tar_path), "extracted_slicer_base")
        os.makedirs(extract_dir, exist_ok=True)
        with tarfile.open(tar_path, "r:gz") as tar:
            tar.extractall(path=extract_dir)
        found_apps = glob.glob(os.path.join(extract_dir, "*", "*.app"))
        if found_apps:
            return found_apps[0]

    # Check standard install path
    if os.path.isdir("/Applications/Slicer.app"):
        print("Using system /Applications/Slicer.app")
        return "/Applications/Slicer.app"

    sys.exit(
        "ERROR: Base Slicer.app not found.\n"
        "Please provide --slicer-app /path/to/Slicer.app or build/package Slicer first."
    )


def relocate_injected_macho(filepath, extra_rpaths):
    """
    Relocate a newly injected Mach-O binary:
      - Set dylib ID to @rpath/<basename>
      - Rewrite external dependencies to @rpath/<dep_base> or @rpath/Frameworks/...
      - Standardize Python dependencies to @rpath/lib/libpython3.12.dylib
      - Strip any host build-tree LC_RPATHs (/opt/..., /Users/...)
      - Inject standard relative LC_RPATH search entries
      - Re-sign with ad-hoc signature
    """
    if not is_macho(filepath):
        return

    basename = os.path.basename(filepath)

    # 1. Update ID for dylibs
    if filepath.endswith(".dylib"):
        try:
            subprocess.check_call(
                ["install_name_tool", "-id", f"@rpath/{basename}", filepath],
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass

    # 2. Inspect dependencies and redirect
    try:
        otool_out = subprocess.check_output(["otool", "-L", filepath]).decode()
    except Exception:
        return

    changes = []
    for line in otool_out.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        dep = line.split()[0]
        dep_base = os.path.basename(dep)

        # Never relocate system frameworks or libraries
        if dep.startswith(("/System", "/usr/lib")):
            continue

        if "libpython3.12" in dep_base:
            target_dep = "@rpath/lib/libpython3.12.dylib"
            if dep != target_dep:
                changes.extend(["-change", dep, target_dep])
        elif ".framework/" in dep:
            match = re.search(r"([^/]+\.framework/.*)$", dep)
            if match:
                target_fw = f"@rpath/Frameworks/{match.group(1)}"
                if dep != target_fw:
                    changes.extend(["-change", dep, target_fw])
        elif (
            "/opt/slicer" in dep
            or "/opt/homebrew" in dep
            or "/Users/" in dep
            or not dep.startswith("@")
        ):
            target_dep = f"@rpath/{dep_base}"
            if dep != target_dep:
                changes.extend(["-change", dep, target_dep])

    if changes:
        try:
            subprocess.check_call(
                ["install_name_tool"] + changes + [filepath],
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError:
            pass

    # 3. Strip host build-tree LC_RPATH entries
    try:
        load_cmds = subprocess.check_output(["otool", "-l", filepath]).decode()
    except Exception:
        load_cmds = ""

    existing_rpaths = set()
    rpaths_to_delete = []
    for line in load_cmds.splitlines():
        line = line.strip()
        if "path " in line and "(offset " in line:
            rp = line.split("path ")[1].split(" (offset")[0].strip()
            existing_rpaths.add(rp)
            if rp.startswith(("/opt/", "/Users/")):
                rpaths_to_delete.append(rp)

    if rpaths_to_delete:
        del_changes = []
        for rp in set(rpaths_to_delete):
            del_changes.extend(["-delete_rpath", rp])
        try:
            subprocess.check_call(
                ["install_name_tool"] + del_changes + [filepath],
                stderr=subprocess.DEVNULL,
            )
            for rp in rpaths_to_delete:
                existing_rpaths.discard(rp)
        except subprocess.CalledProcessError:
            pass

    # 4. Add relative LC_RPATH entries
    for rp in extra_rpaths:
        if rp not in existing_rpaths:
            try:
                subprocess.check_call(
                    ["install_name_tool", "-add_rpath", rp, filepath],
                    stderr=subprocess.DEVNULL,
                )
                existing_rpaths.add(rp)
            except subprocess.CalledProcessError:
                pass

    # 5. Ad-hoc codesign
    try:
        subprocess.check_call(
            ["codesign", "--force", "--sign", "-", filepath],
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError:
        pass


def main():
    parser = argparse.ArgumentParser(description="Package standalone SlicerROS2.app for macOS (arm64)")
    parser.add_argument("--slicer-app", default=None, help="Path to base Slicer.app bundle")
    parser.add_argument("--workspace", default=None, help="Path to slicer_ros2_module workspace")
    parser.add_argument("--build-dir", default=None, help="Path to module build dir (defaults to <workspace>/build)")
    parser.add_argument("--slicer-dir", default=None, help="Path to Slicer-build dir")
    parser.add_argument("--output-dir", default=None, help="Output directory for generated app and dmg")
    parser.add_argument("--app-name", default="SlicerROS2.app", help="Output application bundle name")
    parser.add_argument("--no-dmg", action="store_true", help="Skip DMG creation")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.realpath(__file__))
    workspace = os.path.abspath(args.workspace or os.path.join(script_dir, ".."))
    build_dir = os.path.abspath(args.build_dir or os.path.join(workspace, "build"))
    output_dir = os.path.abspath(args.output_dir or os.path.join(workspace, "dist"))
    slicer_dir = args.slicer_dir or os.environ.get("SLICER_DIR", "/opt/slicer/Slicer-SuperBuild/Slicer-build")

    minimal_ros_install = os.path.join(build_dir, "minimal_ros2-install")
    if not os.path.isdir(minimal_ros_install):
        sys.exit(f"ERROR: minimal_ros2-install not found at {minimal_ros_install}")

    os.makedirs(output_dir, exist_ok=True)

    print("============================================================")
    print("  SlicerROS2 Standalone macOS Application Packager")
    print("============================================================")
    print(f"  Workspace    : {workspace}")
    print(f"  Build Dir    : {build_dir}")
    print(f"  Output Dir   : {output_dir}")
    print(f"  Target App   : {args.app_name}")
    print("============================================================")

    # 1. Locate base Slicer.app
    base_app = find_base_slicer_app(args.slicer_app, slicer_dir, "/opt/slicer/Slicer-SuperBuild")
    target_app = os.path.join(output_dir, args.app_name)

    print(f"\n==> Staging clean application bundle at {target_app}...")
    if os.path.exists(target_app):
        shutil.rmtree(target_app)
    shutil.copytree(base_app, target_app, symlinks=True)

    # Grant write permissions across staged bundle
    subprocess.call(["chmod", "-R", "u+w", target_app])

    # Detect Slicer version folder (e.g. Slicer-5.13)
    lib_dir = os.path.join(target_app, "Contents", "lib")
    slicer_ver_dir = None
    for entry in os.listdir(lib_dir):
        if entry.startswith("Slicer-"):
            slicer_ver_dir = os.path.join(lib_dir, entry)
            slicer_version = entry.split("Slicer-")[1]
            break

    if not slicer_ver_dir:
        slicer_version = "5.13"
        slicer_ver_dir = os.path.join(lib_dir, f"Slicer-{slicer_version}")
        os.makedirs(slicer_ver_dir, exist_ok=True)

    print(f"Detected Slicer internal version directory: Slicer-{slicer_version}")

    qt_loadable_dir = os.path.join(slicer_ver_dir, "qt-loadable-modules")
    qt_scripted_dir = os.path.join(slicer_ver_dir, "qt-scripted-modules")
    site_packages_dir = os.path.join(
        target_app, "Contents", "lib", "Python", "lib", "python3.12", "site-packages"
    )
    os.makedirs(qt_loadable_dir, exist_ok=True)
    os.makedirs(qt_scripted_dir, exist_ok=True)
    os.makedirs(site_packages_dir, exist_ok=True)

    injected_macho_files = set()

    # 2. Inject SlicerROS2 Loadable Modules
    print("\n==> Injecting SlicerROS2 module binaries...")
    module_files = [
        "libqSlicerROS2Module.dylib",
        "libvtkSlicerROS2ModuleLogic.dylib",
        "libvtkSlicerROS2ModuleMRML.dylib",
        "vtkSlicerROS2ModuleLogicPython.so",
        "vtkSlicerROS2ModuleMRMLPython.so",
    ]
    for mod in module_files:
        src = None
        for cand in glob.glob(os.path.join(build_dir, "**", mod), recursive=True):
            if os.path.isfile(cand):
                src = cand
                break
        if not src:
            sys.exit(f"ERROR: Could not find required module file {mod} in {build_dir}")
        print(f"  Copying {mod} -> qt-loadable-modules/")
        dst = copy_file_or_symlink(src, qt_loadable_dir)
        injected_macho_files.add(dst)

    # Inject tests
    tests_py = os.path.join(workspace, "Testing", "Python", "ROS2Tests.py")
    if os.path.isfile(tests_py):
        copy_file_or_symlink(tests_py, qt_scripted_dir)

    # 3. Inject minimal_ros2 and third-party dynamic libraries
    print("\n==> Injecting minimal_ros2 runtime libraries...")
    for f in glob.glob(os.path.join(minimal_ros_install, "lib", "*.dylib")):
        d1 = copy_file_or_symlink(f, slicer_ver_dir)
        d2 = copy_file_or_symlink(f, lib_dir)
        injected_macho_files.add(d1)
        injected_macho_files.add(d2)

    # Vendor libraries (e.g. console_bridge)
    cb_lib = os.path.join(minimal_ros_install, "opt", "console_bridge_vendor", "lib")
    if os.path.isdir(cb_lib):
        for f in glob.glob(os.path.join(cb_lib, "*.dylib")):
            d1 = copy_file_or_symlink(f, slicer_ver_dir)
            d2 = copy_file_or_symlink(f, lib_dir)
            injected_macho_files.add(d1)
            injected_macho_files.add(d2)

    # Third-party runtime libraries with full real binary copying & symlinks
    print("\n==> Injecting third-party runtime libraries (OpenSSL, spdlog, fmt, libyaml)...")
    thirdparty_configs = [
        ("/opt/homebrew/opt/openssl@3/lib/libssl.3.dylib", ["libssl.dylib", "libssl.3.dylib"]),
        ("/opt/homebrew/opt/openssl@3/lib/libcrypto.3.dylib", ["libcrypto.dylib", "libcrypto.3.dylib"]),
        ("/opt/homebrew/opt/spdlog/lib/libspdlog.1.17.dylib", ["libspdlog.dylib", "libspdlog.1.dylib", "libspdlog.1.17.dylib"]),
        ("/opt/homebrew/opt/fmt/lib/libfmt.12.dylib", ["libfmt.dylib", "libfmt.12.dylib"]),
        ("/opt/homebrew/opt/libyaml/lib/libyaml-0.2.dylib", ["libyaml.dylib", "libyaml-0.2.dylib"]),
    ]
    for src_file, aliases in thirdparty_configs:
        if os.path.exists(src_file):
            created = copy_shared_lib_with_symlinks(src_file, [slicer_ver_dir, lib_dir], alias_names=aliases)
            for c in created:
                injected_macho_files.add(c)

    # In macOS dyld without DYLD_LIBRARY_PATH, rcutils_load_shared_library dynamically dlopens
    # "librcl_logging_spdlog.dylib" by bare name. Symlink it into Contents/, bin/, and MacOS/
    for loc in [
        os.path.join(target_app, "Contents", "librcl_logging_spdlog.dylib"),
        os.path.join(target_app, "Contents", "bin", "librcl_logging_spdlog.dylib"),
        os.path.join(target_app, "Contents", "MacOS", "librcl_logging_spdlog.dylib"),
    ]:
        if os.path.exists(loc) or os.path.islink(loc):
            os.remove(loc)
        rel_target = os.path.relpath(os.path.join(lib_dir, "librcl_logging_spdlog.dylib"), os.path.dirname(loc))
        os.symlink(rel_target, loc)

    # 4. Inject Qt plugins (Cocoa QPA, TLS, SQLite drivers) & supporting frameworks
    print("\n==> Injecting essential Qt plugins and supporting frameworks...")
    # Find Homebrew qtbase plugins
    hb_plugin_candidates = glob.glob("/opt/homebrew/Cellar/qtbase/*/share/qt/plugins") + [
        "/opt/homebrew/share/qt/plugins",
        "/opt/homebrew/opt/qt/share/qt/plugins",
    ]
    hb_plugins_dir = None
    for cand in hb_plugin_candidates:
        if os.path.isdir(cand) and os.path.isfile(os.path.join(cand, "platforms", "libqcocoa.dylib")):
            hb_plugins_dir = cand
            break

    if hb_plugins_dir:
        print(f"  Found Qt plugins source: {hb_plugins_dir}")
        qt_plugins_dst = os.path.join(lib_dir, "QtPlugins")
        for sub in ["platforms", "tls", "sqldrivers"]:
            src_sub = os.path.join(hb_plugins_dir, sub)
            dst_sub = os.path.join(qt_plugins_dst, sub)
            if os.path.isdir(src_sub):
                os.makedirs(dst_sub, exist_ok=True)
                for f in glob.glob(os.path.join(src_sub, "*.dylib")):
                    dst_file = copy_file_or_symlink(f, dst_sub)
                    injected_macho_files.add(dst_file)

        # QtDBus framework (required by Homebrew QtGui.framework)
        dbus_fw = "/opt/homebrew/opt/qt/lib/QtDBus.framework"
        dst_fw = os.path.join(target_app, "Contents", "Frameworks", "QtDBus.framework")
        if os.path.isdir(dbus_fw) and not os.path.exists(dst_fw):
            print("  Bundling QtDBus.framework...")
            shutil.copytree(dbus_fw, dst_fw, symlinks=True)
            for root, _, files in os.walk(dst_fw):
                for f in files:
                    p = os.path.join(root, f)
                    if is_macho(p):
                        injected_macho_files.add(p)

        # libdbus-1.3.dylib (required by QtDBus.framework)
        for cand in glob.glob("/opt/homebrew/Cellar/dbus/*/lib/libdbus-1.3.dylib") + ["/opt/homebrew/opt/dbus/lib/libdbus-1.3.dylib"]:
            if os.path.exists(cand):
                created = copy_shared_lib_with_symlinks(cand, [lib_dir], alias_names=["libdbus-1.dylib"])
                for c in created:
                    injected_macho_files.add(c)
                break

        # libbrotlicommon (required by libbrotlidec in Qt)
        for cand in glob.glob("/opt/homebrew/Cellar/brotli/*/lib/libbrotlicommon.1.dylib") + ["/opt/homebrew/opt/brotli/lib/libbrotlicommon.1.dylib"]:
            if os.path.exists(cand):
                created = copy_shared_lib_with_symlinks(cand, [lib_dir], alias_names=["libbrotlicommon.dylib"])
                for c in created:
                    injected_macho_files.add(c)
                break
    else:
        print("  Warning: Homebrew Qt plugins not found. Skipping plugin injection.")

    # 5. Fix bare dependencies in CTK modules (libPythonQt.dylib)
    print("\n==> Fixing bare library references in CTK modules...")
    pyqt_symlink = os.path.join(slicer_ver_dir, "libPythonQt.dylib")
    if not os.path.exists(pyqt_symlink) and not os.path.islink(pyqt_symlink):
        os.symlink("../libPythonQt.dylib", pyqt_symlink)

    for p in glob.glob(os.path.join(lib_dir, "libCTK*.dylib")) + glob.glob(os.path.join(slicer_ver_dir, "*PythonQt.so")):
        if is_macho(p):
            try:
                otool_out = subprocess.check_output(["otool", "-L", p]).decode()
                changes = []
                for line in otool_out.splitlines()[1:]:
                    dep = line.strip().split()[0]
                    if dep == "libPythonQt.dylib":
                        changes.extend(["-change", "libPythonQt.dylib", "@rpath/libPythonQt.dylib"])
                if changes:
                    subprocess.check_call(["install_name_tool"] + changes + [p], stderr=subprocess.DEVNULL)
                    injected_macho_files.add(p)
            except Exception:
                pass

    # 6. Inject ROS 2 Python site-packages (rclpy, messages, and runtime deps)
    print("\n==> Injecting ROS 2 Python site-packages into Slicer Python...")
    minimal_sp = os.path.join(minimal_ros_install, "lib", "python3.12", "site-packages")
    if os.path.isdir(minimal_sp):
        copied_sp = copy_tree_preserving_symlinks(minimal_sp, site_packages_dir)
        for c in copied_sp:
            if is_macho(c):
                injected_macho_files.add(c)

    venv_sp = os.path.join(
        build_dir, "minimal_ros2-prefix", "src", "minimal_ros2", ".venv", "lib", "python3.12", "site-packages"
    )
    venv_pkgs = [
        "em.py", "lark", "yaml", "_yaml", "catkin_pkg", "packaging",
        "pyparsing", "dateutil", "six.py"
    ]
    if os.path.isdir(venv_sp):
        for pkg in venv_pkgs:
            src_pkg = os.path.join(venv_sp, pkg)
            dst_pkg = os.path.join(site_packages_dir, pkg)
            if os.path.isdir(src_pkg):
                if os.path.exists(dst_pkg):
                    shutil.rmtree(dst_pkg)
                shutil.copytree(src_pkg, dst_pkg)
                for root, _, files in os.walk(dst_pkg):
                    for f in files:
                        p = os.path.join(root, f)
                        if is_macho(p):
                            injected_macho_files.add(p)
            elif os.path.isfile(src_pkg):
                shutil.copy2(src_pkg, dst_pkg)
                if is_macho(dst_pkg):
                    injected_macho_files.add(dst_pkg)

    # 7. Fix dependencies, RPATHs, and ad-hoc sign all injected Mach-O binaries
    print(f"\n==> Relocating, stripping build RPATHs, and signing {len(injected_macho_files)} injected binaries...")

    common_rpaths = [
        "@loader_path",
        "@loader_path/..",
        "@loader_path/../..",
        "@loader_path/../../..",
        "@loader_path/../../../..",
        "@loader_path/../../../../..",
        "@loader_path/../../../../../..",
        "@executable_path/../",
        "@executable_path/../Frameworks",
        "@executable_path/../lib",
        f"@executable_path/../lib/Slicer-{slicer_version}",
        "@executable_path/../lib/Python/lib",
    ]

    count_relocated = 0
    for p in sorted(injected_macho_files):
        if is_macho(p):
            relocate_injected_macho(p, common_rpaths)
            count_relocated += 1

    print(f"  Successfully relocated {count_relocated} injected Mach-O binaries.")

    # 8. Headless Verification Tests
    print("\n==> Running headless verification tests...")
    python_slicer_exe = os.path.join(target_app, "Contents", "bin", "PythonSlicer")
    slicer_exe = os.path.join(target_app, "Contents", "MacOS", "Slicer")

    # Clean up transient settings before tests
    transient_settings = os.path.join(target_app, "Contents", "slicer.org")
    if os.path.exists(transient_settings):
        shutil.rmtree(transient_settings)

    # Scratch test script
    verify_script = os.path.join(output_dir, "verify_bundle.py")
    with open(verify_script, "w") as f:
        f.write(
            "import sys\n"
            "import slicer\n"
            "print('--- Python runtime:', sys.version.split()[0])\n"
            "if hasattr(slicer, 'app') and slicer.app:\n"
            "    print('--- Slicer application version:', slicer.app.applicationVersion)\n"
            "    has_ros2 = 'ros2' in dir(slicer.modules) or 'ROS2' in dir(slicer.modules)\n"
            "    print('--- SlicerROS2 module loaded:', has_ros2)\n"
            "    assert has_ros2, 'SlicerROS2 module was not loaded!'\n"
            "import rclpy\n"
            "print('--- rclpy module location:', rclpy.__file__)\n"
            "rclpy.init()\n"
            "node = rclpy.create_node('packaged_app_verifier')\n"
            "print('--- Created ROS 2 node:', node.get_name())\n"
            "from std_msgs.msg import String\n"
            "pub = node.create_publisher(String, 'verify_topic', 10)\n"
            "print('--- Created ROS 2 publisher on topic:', pub.topic_name)\n"
            "rclpy.shutdown()\n"
            "print('--- rclpy shutdown SUCCESS')\n"
            "if hasattr(slicer, 'app') and slicer.app:\n"
            "    slicer.app.quit()\n"
            "sys.exit(0)\n"
        )

    test_env = dict(os.environ)
    test_env["PYTHONDONTWRITEBYTECODE"] = "1"

    # Test 1: PythonSlicer Standalone Runtime
    print("  [Test 1/2] Verifying PythonSlicer standalone runtime...")
    try:
        out1 = subprocess.check_output(
            [python_slicer_exe, verify_script],
            env=test_env,
            stderr=subprocess.STDOUT,
            timeout=30,
        ).decode()
        for line in out1.splitlines():
            if "---" in line:
                print(f"    {line}")
        print("  -> PythonSlicer test PASSED!")
    except subprocess.CalledProcessError as e:
        print(f"  -> ERROR in PythonSlicer test (exit code {e.returncode}):\n{e.output.decode()}")
        sys.exit(1)

    # Test 2: Full Slicer Headless Mode (Modules + Python ROS 2)
    print("  [Test 2/2] Verifying full 3D Slicer application runtime...")
    try:
        out2 = subprocess.check_output(
            [slicer_exe, "--testing", "--no-main-window", "--python-script", verify_script],
            env=test_env,
            stderr=subprocess.STDOUT,
            timeout=40,
        ).decode()
        for line in out2.splitlines():
            if "---" in line:
                print(f"    {line}")
        print("  -> Full 3D Slicer test PASSED!")
    except subprocess.CalledProcessError as e:
        print(f"  -> ERROR in Slicer test (exit code {e.returncode}):\n{e.output.decode()}")
        sys.exit(1)

    # Clean up verification scratch files and pycache
    if os.path.exists(verify_script):
        os.remove(verify_script)
    if os.path.exists(transient_settings):
        shutil.rmtree(transient_settings)
    subprocess.call(["find", target_app, "-name", "__pycache__", "-exec", "rm", "-rf", "{}", "+"])
    subprocess.call(["find", target_app, "-name", "*.pyc", "-delete"])

    # 9. Deep Ad-Hoc Code Signing and Integrity Verification of the Application Bundle
    print("\n==> Performing deep ad-hoc code-signing of SlicerROS2.app...")
    subprocess.call(["chmod", "-R", "u+w", target_app])
    subprocess.call(["xattr", "-cr", target_app])
    subprocess.check_call(["codesign", "--force", "--deep", "--sign", "-", target_app])
    subprocess.check_call(["codesign", "--verify", "--deep", "--strict", target_app])
    print("  Code-signing seal verified successfully.")

    # 10. Create Distribution Disk Image (.dmg)
    if not args.no_dmg:
        dmg_name = "SlicerROS2-5.13.0-macos-arm64.dmg"
        dmg_path = os.path.join(output_dir, dmg_name)
        print(f"\n==> Creating distribution disk image (.dmg): {dmg_path}...")
        if os.path.exists(dmg_path):
            os.remove(dmg_path)

        subprocess.check_call([
            "hdiutil", "create",
            "-volname", "SlicerROS2-arm64",
            "-srcfolder", target_app,
            "-ov", "-format", "UDZO",
            dmg_path,
        ])
        print(f"==> Successfully generated disk image: {dmg_path}")
        print(f"    Size: {os.path.getsize(dmg_path) / (1024*1024):.1f} MB")


    print("\n============================================================")
    print("  Packaging Complete!")
    print(f"  App Bundle : {target_app}")
    if not args.no_dmg:
        print(f"  DMG Image  : {dmg_path}")
    print("============================================================")


if __name__ == "__main__":
    main()
