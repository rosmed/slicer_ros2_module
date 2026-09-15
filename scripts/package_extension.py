#!/usr/bin/env python3
"""
package_extension.py

Creates a standalone binary 3D Slicer extension package (.tar.gz / .tgz)
for SlicerROS2, bundling all runtime dependencies from minimal_ros2 and
necessary third-party libraries (OpenSSL, spdlog, fmt, libyaml, console_bridge).
Performs Mach-O relocation (install names, RPATHs) and ad-hoc code-signing
for macOS (arm64 / x86_64).
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import tarfile
from datetime import datetime


def is_macho(filepath):
    """Check if the file is a Mach-O binary."""
    if os.path.islink(filepath) or not os.path.isfile(filepath):
        return False
    try:
        with open(filepath, "rb") as f:
            magic = f.read(4)
            # Mach-O magic numbers: 32-bit/64-bit, little/big endian, FAT
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


def get_git_info(repo_dir):
    """Retrieve git short hash and date."""
    try:
        rev = (
            subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"], cwd=repo_dir
            )
            .decode()
            .strip()
        )
    except Exception:
        rev = "unknown"
    date_str = datetime.now().strftime("%Y-%m-%d")
    return rev, date_str


def get_slicer_info(slicer_bin=None):
    """Query Slicer for revision, os, architecture, and version."""
    info = {
        "revision": "34939",
        "os": "macosx",
        "arch": "arm64",
        "version": "5.13",
    }
    if slicer_bin and os.path.exists(slicer_bin):
        try:
            code = (
                "import sys, slicer; "
                "em = slicer.app.extensionsManagerModel(); "
                "print('SLICER_INFO:', em.slicerRevision, em.slicerOs, em.slicerArch, "
                "f'{slicer.app.majorVersion}.{slicer.app.minorVersion}'); "
                "sys.exit(0)"
            )
            out = subprocess.check_output(
                [
                    slicer_bin,
                    "--no-splash",
                    "--no-main-window",
                    "--exit-after-startup",
                    "--python-code",
                    code,
                ],
                stderr=subprocess.STDOUT,
                timeout=15,
            ).decode()
            for line in out.splitlines():
                if "SLICER_INFO:" in line:
                    parts = line.split("SLICER_INFO:")[1].strip().split()
                    if len(parts) >= 4:
                        info["revision"] = parts[0]
                        info["os"] = parts[1]
                        info["arch"] = parts[2]
                        info["version"] = parts[3]
                    break
        except Exception as e:
            print(f"Notice: Using default Slicer metadata ({e})", flush=True)
    return info


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
        shutil.copy2(src, dst_path)
    return dst_path


def copy_tree_preserving_symlinks(src_dir, dst_dir):
    """Copy directory tree preserving symlinks."""
    os.makedirs(dst_dir, exist_ok=True)
    for root, dirs, files in os.walk(src_dir):
        rel = os.path.relpath(root, src_dir)
        target_root = os.path.join(dst_dir, rel) if rel != "." else dst_dir
        os.makedirs(target_root, exist_ok=True)
        for d in dirs:
            os.makedirs(os.path.join(target_root, d), exist_ok=True)
        for f in files:
            src_file = os.path.join(root, f)
            copy_file_or_symlink(src_file, target_root)


def relocate_macho_file(filepath, base_dir, all_bundled_basenames, extra_rpaths, slicer_build_dir=None):
    """Set install ID, rewrite dependency paths to @rpath, inject RPATHs, and ad-hoc codesign."""
    if not is_macho(filepath):
        return

    basename = os.path.basename(filepath)
    rel_path = os.path.relpath(filepath, base_dir)

    # 1. Update ID for dylibs
    if filepath.endswith(".dylib"):
        try:
            subprocess.check_call(
                ["install_name_tool", "-id", f"@rpath/{basename}", filepath],
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            print(f"Warning setting id for {rel_path}: {e}")

    # 2. Inspect dependencies
    try:
        otool_out = subprocess.check_output(["otool", "-L", filepath]).decode()
    except Exception as e:
        print(f"Warning inspecting {rel_path}: {e}")
        return

    changes = []
    for line in otool_out.splitlines()[1:]:
        line = line.strip()
        if not line:
            continue
        dep = line.split()[0]
        dep_base = os.path.basename(dep)

        # Check if dependency matches any of our bundled libraries
        if dep_base in all_bundled_basenames:
            if dep != f"@rpath/{dep_base}":
                changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif "/opt/homebrew/opt/openssl" in dep or "/opt/homebrew/Cellar/openssl" in dep:
            changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif "/opt/homebrew/opt/spdlog" in dep:
            changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif "/opt/homebrew/opt/fmt" in dep:
            changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif "/opt/homebrew/opt/libyaml" in dep:
            changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif "console_bridge" in dep:
            changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif "libpython3.12" in dep:
            changes.extend(["-change", dep, "@rpath/libpython3.12.dylib"])
        elif slicer_build_dir and slicer_build_dir in dep:
            # Slicer build tree library -> @rpath/<basename>
            changes.extend(["-change", dep, f"@rpath/{dep_base}"])
        elif re.search(r"/opt/homebrew/.*\.framework/Versions/([^/]+)/([^/]+)", dep):
            # Qt framework
            m = re.search(r"([^/]+\.framework/Versions/([^/]+)/[^/]+)", dep)
            if m:
                changes.extend(["-change", dep, f"@rpath/{m.group(1)}"])

    if changes:
        try:
            subprocess.check_call(
                ["install_name_tool"] + changes + [filepath],
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError as e:
            print(f"Warning running install_name_tool -change on {rel_path}: {e}")

    # 3. Add LC_RPATH entries
    try:
        load_cmds = subprocess.check_output(["otool", "-l", filepath]).decode()
    except Exception:
        load_cmds = ""

    existing_rpaths = set()
    for line in load_cmds.splitlines():
        if "path " in line and "(offset " in line:
            rp = line.split("path ")[1].split(" (offset")[0].strip()
            existing_rpaths.add(rp)

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

    # 4. Ad-hoc codesign
    try:
        subprocess.check_call(
            ["codesign", "--force", "--sign", "-", filepath],
            stderr=subprocess.DEVNULL,
        )
    except subprocess.CalledProcessError as e:
        print(f"Warning codesigning {rel_path}: {e}")


def main():
    parser = argparse.ArgumentParser(description="Package SlicerROS2 binary extension")
    parser.add_argument(
        "--workspace",
        default=os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
        help="Path to slicer_ros2_module workspace",
    )
    parser.add_argument(
        "--build-dir",
        default=None,
        help="Path to build directory (defaults to <workspace>/build)",
    )
    parser.add_argument(
        "--slicer-dir",
        default=None,
        help="Path to Slicer-build directory",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory to save the packaged .tar.gz archive (defaults to <workspace>/dist)",
    )
    args = parser.parse_args()

    workspace = os.path.abspath(args.workspace)
    build_dir = args.build_dir or os.path.join(workspace, "build")
    slicer_dir = args.slicer_dir or os.environ.get(
        "SLICER_DIR", "/Users/anton/devel/slicer/build/Slicer-build"
    )
    output_dir = args.output_dir or os.path.join(workspace, "dist")
    os.makedirs(output_dir, exist_ok=True)

    slicer_bin = os.path.join(slicer_dir, "Slicer")
    slicer_info = get_slicer_info(slicer_bin)
    git_rev, git_date = get_git_info(workspace)

    extension_name = "SlicerROS2"
    slicer_rev = slicer_info["revision"]
    slicer_os = slicer_info["os"]
    slicer_arch = slicer_info["arch"]
    slicer_version = slicer_info["version"]  # 5.13

    archive_basename = f"{slicer_rev}-{slicer_os}-{slicer_arch}-{extension_name}-git{git_rev}-{git_date}"
    print(f"=== Packaging Slicer Extension: {archive_basename} ===")
    print(f"  Workspace: {workspace}")
    print(f"  Build dir: {build_dir}")
    print(f"  Slicer dir: {slicer_dir}")
    print(f"  Slicer rev: {slicer_rev} ({slicer_os}-{slicer_arch}), version: {slicer_version}")

    # Paths to source components
    minimal_ros_install = os.path.join(build_dir, "minimal_ros2-install")
    slicer_mod_build = os.path.join(build_dir, "slicer_ros2_module-build")
    slicer_mod_install = os.path.join(build_dir, "slicer_ros2_module-install")

    if not os.path.isdir(minimal_ros_install):
        sys.exit(f"Error: minimal_ros2-install directory not found at {minimal_ros_install}")

    # Create staging directory
    staging_root = os.path.join(build_dir, "package_staging")
    if os.path.exists(staging_root):
        shutil.rmtree(staging_root)

    archive_top = os.path.join(staging_root, archive_basename)
    extension_payload_root = os.path.join(
        archive_top,
        "Slicer.app",
        "Contents",
        f"Extensions-{slicer_rev}",
        extension_name,
    )

    share_dir = os.path.join(extension_payload_root, "share", f"Slicer-{slicer_version}")
    qt_loadable_dir = os.path.join(
        extension_payload_root, "lib", f"Slicer-{slicer_version}", "qt-loadable-modules"
    )
    qt_scripted_dir = os.path.join(
        extension_payload_root, "lib", f"Slicer-{slicer_version}", "qt-scripted-modules"
    )
    thirdparty_lib_dir = os.path.join(
        extension_payload_root, "lib", f"Slicer-{slicer_version}"
    )
    site_packages_dir = os.path.join(
        extension_payload_root, "lib", "python3.12", "site-packages"
    )

    for d in [
        share_dir,
        qt_loadable_dir,
        qt_scripted_dir,
        thirdparty_lib_dir,
        site_packages_dir,
    ]:
        os.makedirs(d, exist_ok=True)

    # 1. Create SlicerROS2.s4ext
    s4ext_content = f"""# SlicerROS2 extension description file
scm git
scmurl https://github.com/rosmed/slicer_ros2_module.git
scmrevision {git_rev}
category Robotics
contributors Anton Deguet (JHU), SlicerROS2 contributors
homepage https://github.com/rosmed/slicer_ros2_module
description Slicer ROS 2 module providing ROS 2 connectivity, robotics data visualization, transforms, and robot control in 3D Slicer.
status Active
enabled 1
depends
os {slicer_os}
arch {slicer_arch}
slicer_revision {slicer_rev}
"""
    # Write at top of archive (for Slicer archive lister) and in share/Slicer-5.13/
    with open(os.path.join(archive_top, f"{extension_name}.s4ext"), "w") as f:
        f.write(s4ext_content)
    with open(os.path.join(share_dir, f"{extension_name}.s4ext"), "w") as f:
        f.write(s4ext_content)

    print("Created extension description metadata (.s4ext)")

    # 2. Copy Loadable Module Binaries
    print("Copying loadable module binaries...")
    module_lib_candidates = [
        "libqSlicerROS2Module.dylib",
        "libvtkSlicerROS2ModuleLogic.dylib",
        "libvtkSlicerROS2ModuleMRML.dylib",
        "vtkSlicerROS2ModuleLogicPython.so",
        "vtkSlicerROS2ModuleMRMLPython.so",
    ]
    for mod_lib in module_lib_candidates:
        src = None
        for cand_dir in [
            os.path.join(
                slicer_mod_install,
                "Slicer.app",
                "Contents",
                f"Extensions-{slicer_rev}",
                "ROS2",
                "lib",
                f"Slicer-{slicer_version}",
                "qt-loadable-modules",
            ),
            os.path.join(
                slicer_mod_build,
                "Users/anton/devel/slicer/slicer_ros2_module/build/slicer_ros2_module-build/lib",
            ),
            os.path.join(slicer_mod_build, "lib"),
        ]:
            p = os.path.join(cand_dir, mod_lib)
            if os.path.exists(p):
                src = p
                break
        if not src:
            sys.exit(f"Error: Could not find required module file {mod_lib}")
        copy_file_or_symlink(src, qt_loadable_dir)

    # 3. Copy Scripted Modules (ROS2Tests.py)
    print("Copying scripted modules...")
    tests_py = os.path.join(workspace, "Testing", "Python", "ROS2Tests.py")
    if os.path.exists(tests_py):
        copy_file_or_symlink(tests_py, qt_scripted_dir)

    # 4. Copy minimal_ros2 dylibs
    print("Copying minimal_ros2 dynamic libraries...")
    minimal_lib_dir = os.path.join(minimal_ros_install, "lib")
    for f in glob.glob(os.path.join(minimal_lib_dir, "*.dylib")):
        copy_file_or_symlink(f, thirdparty_lib_dir)

    # Copy console_bridge vendor dylibs
    cb_lib_dir = os.path.join(
        minimal_ros_install, "opt", "console_bridge_vendor", "lib"
    )
    if os.path.isdir(cb_lib_dir):
        for f in glob.glob(os.path.join(cb_lib_dir, "*.dylib")):
            copy_file_or_symlink(f, thirdparty_lib_dir)

    # 5. Copy Third-Party Dependencies (OpenSSL, spdlog, fmt, libyaml)
    print("Copying third-party runtime dependencies...")
    thirdparty_sources = [
        ("/opt/homebrew/opt/openssl@3/lib/libssl.3.dylib", "libssl.dylib"),
        ("/opt/homebrew/opt/openssl@3/lib/libcrypto.3.dylib", "libcrypto.dylib"),
        ("/opt/homebrew/opt/spdlog/lib/libspdlog.1.17.dylib", "libspdlog.dylib"),
        ("/opt/homebrew/opt/fmt/lib/libfmt.12.dylib", "libfmt.dylib"),
        ("/opt/homebrew/opt/libyaml/lib/libyaml-0.2.dylib", "libyaml.dylib"),
    ]
    for src_file, symlink_name in thirdparty_sources:
        if not os.path.exists(src_file):
            print(f"Warning: Expected third-party lib {src_file} not found!")
            continue
        dst_file = copy_file_or_symlink(src_file, thirdparty_lib_dir)
        sym_path = os.path.join(thirdparty_lib_dir, symlink_name)
        if not os.path.exists(sym_path) and not os.path.islink(sym_path):
            os.symlink(os.path.basename(dst_file), sym_path)

    # 6. Copy minimal_ros2 Python site-packages
    print("Copying ROS 2 Python site-packages...")
    minimal_site_packages = os.path.join(
        minimal_ros_install, "lib", "python3.12", "site-packages"
    )
    if os.path.isdir(minimal_site_packages):
        copy_tree_preserving_symlinks(minimal_site_packages, site_packages_dir)

    # Also copy necessary runtime pure-python packages from minimal_ros2 .venv
    venv_candidates = [
        os.path.join(
            build_dir,
            "minimal_ros2-prefix",
            "src",
            "minimal_ros2",
            ".venv",
            "lib",
            "python3.12",
            "site-packages",
        ),
        os.path.join(workspace, "..", ".venv", "lib", "python3.12", "site-packages"),
    ]
    venv_pkgs = [
        "em.py",
        "lark",
        "yaml",
        "_yaml",
        "catkin_pkg",
        "packaging",
        "pyparsing",
        "dateutil",
        "six.py",
    ]
    for venv_sp in venv_candidates:
        if os.path.isdir(venv_sp):
            print(f"Bundling runtime pure-Python packages from {venv_sp}...")
            for pkg in venv_pkgs:
                src_pkg = os.path.join(venv_sp, pkg)
                dst_pkg = os.path.join(site_packages_dir, pkg)
                if os.path.isdir(src_pkg):
                    if os.path.exists(dst_pkg):
                        shutil.rmtree(dst_pkg)
                    shutil.copytree(src_pkg, dst_pkg)
                elif os.path.isfile(src_pkg):
                    shutil.copy2(src_pkg, dst_pkg)
            break

    # Collect all bundled filenames and basenames for relocation matching
    all_bundled_basenames = set()
    for root, dirs, files in os.walk(extension_payload_root):
        for f in files:
            all_bundled_basenames.add(f)

    print(f"Bundled {len(all_bundled_basenames)} total files.")

    # 7. Relocate Mach-O Binaries & Code Sign
    print("Relocating and ad-hoc signing all Mach-O binaries...")

    # RPATH configurations
    loadable_extra_rpaths = [
        "@loader_path",
        "@loader_path/..",
        "@executable_path/../lib/Slicer-" + slicer_version,
        "@executable_path/../Frameworks",
        "@executable_path/../lib/Python/lib",
        # Local build tree fallback RPATHs so this build can test the package directly:
        os.path.join(slicer_dir, "bin"),
        os.path.join(slicer_dir, "..", "VTK-build", "lib"),
        os.path.join(slicer_dir, "..", "ITK-build", "lib"),
        os.path.join(slicer_dir, "..", "CTK-build", "CTK-build", "bin"),
        os.path.join(
            slicer_dir, "..", "CTK-build", "CMakeExternals", "Install", "lib"
        ),
        os.path.join(slicer_dir, "..", "teem-build", "bin"),
        os.path.join(slicer_dir, "..", "LibArchive-install", "lib"),
        os.path.join(slicer_dir, "..", "python-install", "lib"),
        os.path.join(
            slicer_dir,
            "..",
            "SlicerExecutionModel-build",
            "ModuleDescriptionParser",
            "bin",
        ),
        os.path.join(slicer_dir, "..", "OpenSSL"),
        "/opt/homebrew/lib",
        "/opt/homebrew/opt/qtbase/lib",
        "/opt/homebrew/opt/qttools/lib",
        "/opt/homebrew/opt/qtsvg/lib",
        "/opt/homebrew/opt/qtmultimedia/lib",
        "/opt/homebrew/opt/qtwebengine/lib",
        "/opt/homebrew/opt/qtpositioning/lib",
        "/opt/homebrew/opt/qtwebchannel/lib",
        "/opt/homebrew/opt/qtdeclarative/lib",
        "/opt/homebrew/opt/qt5compat/lib",
        "/opt/homebrew/opt/qtscxml/lib",
    ]

    thirdparty_extra_rpaths = [
        "@loader_path",
        "@loader_path/..",
        "@executable_path/../lib/Slicer-" + slicer_version,
        "@executable_path/../Frameworks",
        "@executable_path/../lib/Python/lib",
    ]

    python_extra_rpaths = [
        "@loader_path",
        "@loader_path/..",
        "@loader_path/../..",
        "@loader_path/../../..",
        "@loader_path/../../../..",
        "@loader_path/../../../../../..",
        f"@loader_path/../../Slicer-{slicer_version}",
        f"@loader_path/../../../Slicer-{slicer_version}",
        f"@loader_path/../../../../Slicer-{slicer_version}",
        f"@loader_path/../../../../../Slicer-{slicer_version}",
        f"@loader_path/../../../../../../Slicer-{slicer_version}",
        "@executable_path/../lib/Slicer-" + slicer_version,
        "@executable_path/../Frameworks",
        "@executable_path/../lib/Python/lib",
    ]

    # Process qt-loadable-modules
    for root, _, files in os.walk(qt_loadable_dir):
        for f in files:
            p = os.path.join(root, f)
            if not os.path.islink(p) and is_macho(p):
                relocate_macho_file(
                    p,
                    extension_payload_root,
                    all_bundled_basenames,
                    loadable_extra_rpaths,
                    slicer_build_dir=os.path.dirname(slicer_dir),
                )

    # Process thirdparty_lib_dir
    for root, _, files in os.walk(thirdparty_lib_dir):
        if "qt-loadable-modules" in root or "qt-scripted-modules" in root:
            continue
        for f in files:
            p = os.path.join(root, f)
            if not os.path.islink(p) and is_macho(p):
                relocate_macho_file(
                    p,
                    extension_payload_root,
                    all_bundled_basenames,
                    thirdparty_extra_rpaths,
                    slicer_build_dir=os.path.dirname(slicer_dir),
                )

    # Process python site-packages (.so and .dylib)
    for root, _, files in os.walk(site_packages_dir):
        for f in files:
            p = os.path.join(root, f)
            if not os.path.islink(p) and is_macho(p):
                relocate_macho_file(
                    p,
                    extension_payload_root,
                    all_bundled_basenames,
                    python_extra_rpaths,
                    slicer_build_dir=os.path.dirname(slicer_dir),
                )

    print("Relocation and signing complete.")

    # 8. Create Archive (.tar.gz and .tgz)
    tar_gz_path = os.path.join(output_dir, f"{archive_basename}.tar.gz")
    tgz_path = os.path.join(output_dir, f"{archive_basename}.tgz")
    convenience_path = os.path.join(output_dir, f"{extension_name}-{slicer_os}-{slicer_arch}.tar.gz")

    print(f"Creating archive {tar_gz_path}...")
    with tarfile.open(tar_gz_path, "w:gz") as tar:
        tar.add(archive_top, arcname=archive_basename)

    shutil.copy2(tar_gz_path, tgz_path)
    shutil.copy2(tar_gz_path, convenience_path)

    tar_size_mb = os.path.getsize(tar_gz_path) / (1024 * 1024)
    print(f"\nSUCCESS: Extension archive created successfully!")
    print(f"  Archive: {tar_gz_path} ({tar_size_mb:.2f} MB)")
    print(f"  TGZ:     {tgz_path}")
    print(f"  Alias:   {convenience_path}")


if __name__ == "__main__":
    main()
