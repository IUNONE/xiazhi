#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble workspace setup. Never connects to a robot.
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
STATE_DIR="$ROOT/.setup-state"
FROM_STEP=1
TO_STEP=10
RESUME=0
ROBOT_IP=""
ROBOT_PORT=""
CURRENT_STEP=0
TOTAL=10
NAMES=("" "official submodules" "system dependencies" "RealMan SDK library" "rosdep dependencies" "RealMan connection configuration" "rm_ros_interfaces" "RealMan and Orbbec drivers" "Orbbec udev rules" "workspace verification" "project venv")

usage() {
    cat <<'EOF'
Usage: bash scripts/setup.sh [options]
  --list                 List numbered steps; no changes
  --from-step N          Start at step N (1..10)
  --to-step N            Stop after step N
  --resume               Restart at the last failed/interrupted step
  --robot-ip IP          Set the driver's arm_ip (otherwise preserve existing value)
  --robot-port PORT      Set the driver's tcp_port (otherwise preserve existing value)
  --help                 Show this help

Run as a regular user in a clean Ubuntu 22.04 ROS Humble shell, outside Conda.
ROS Humble must already be installed at /opt/ros/humble. Privileged steps use sudo.
Already-installed steps are verified and skipped. Build stamps include source hashes.
Resume progress and logs are kept under .setup-state/ (not committed).
EOF
}
while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --list) for ((i=1;i<=TOTAL;i++)); do printf '[%s/%s] %s\n' "$i" "$TOTAL" "${NAMES[$i]}"; done; exit 0 ;;
        --resume) RESUME=1; shift ;;
        --from-step|--to-step|--robot-ip|--robot-port)
            (($# >= 2)) || { printf 'Missing value for %s\n' "$1" >&2; exit 2; }
            case "$1" in
                --from-step) FROM_STEP=$2 ;;
                --to-step) TO_STEP=$2 ;;
                --robot-ip) ROBOT_IP=$2 ;;
                --robot-port) ROBOT_PORT=$2 ;;
            esac
            shift 2 ;;
        *) printf 'Unknown argument: %s\n' "$1" >&2; usage; exit 2 ;;
    esac
done
[[ "$FROM_STEP" =~ ^([1-9]|10)$ && "$TO_STEP" =~ ^([1-9]|10)$ ]] || { echo 'Step must be 1..10' >&2; exit 2; }
if ((RESUME)); then
    [[ "$FROM_STEP" == 1 ]] || { echo '--resume and --from-step cannot be combined' >&2; exit 2; }
    if [[ -f "$STATE_DIR/next-step" ]]; then
        read -r FROM_STEP < "$STATE_DIR/next-step"
        [[ "$FROM_STEP" =~ ^([1-9]|10|11)$ ]] || { echo 'Invalid resume checkpoint' >&2; exit 2; }
    fi
fi
if [[ "$FROM_STEP" == 11 ]]; then
    echo 'Previous setup completed. Run without --resume to recheck installed state.'
    exit 0
fi
((FROM_STEP <= TO_STEP)) || { echo '--from-step exceeds --to-step' >&2; exit 2; }
if [[ -n "${CONDA_PREFIX:-}" || -n "${CONDA_DEFAULT_ENV:-}" || "${CONDA_SHLVL:-0}" =~ ^[0-9]*[1-9][0-9]*$ ]]; then
    printf '❌ 检测到 Conda 环境：%s，已停止安装。ROS 安装需要系统 Python。\n' "${CONDA_DEFAULT_ENV:-${CONDA_PREFIX:-unknown}}" >&2
    echo '请在当前终端执行 conda deactivate，再重新运行安装命令；嵌套环境需要逐层退出。' >&2
    exit 1
fi
[[ "$(uname -s)" == Linux && -f /etc/os-release ]] || { echo 'Ubuntu 22.04 required' >&2; exit 1; }
# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Ubuntu 22.04 required' >&2; exit 1; }
[[ -z "${VIRTUAL_ENV:-}" && -z "${PYTHONHOME:-}" ]] || { echo 'Deactivate venv and unset PYTHONHOME first' >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html' >&2; exit 1; }
[[ "$(/usr/bin/python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')" == 3.10 ]] || { echo 'System Python 3.10 required' >&2; exit 1; }
((EUID != 0)) || { echo 'Run as a regular user; privileged steps invoke sudo' >&2; exit 1; }
mkdir -p "$STATE_DIR"
exec 9> "$STATE_DIR/setup.lock"
flock -n 9 || { echo 'Another setup is running' >&2; exit 1; }
LOG="$STATE_DIR/setup-$(date +%Y%m%d-%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
checkpoint() { printf '%s\n' "$1" > "$STATE_DIR/next-step.tmp"; mv "$STATE_DIR/next-step.tmp" "$STATE_DIR/next-step"; }
failed() {
    local code=$?
    trap - ERR
    printf '\n[%s/%s] ❌ Failed: %s (exit %s)\n' "$CURRENT_STEP" "$TOTAL" "${NAMES[$CURRENT_STEP]}" "$code"
    printf 'Fix the error, then run: bash scripts/setup.sh --resume [same --robot-ip/--robot-port options]\nLog: %s\n' "$LOG"
    exit "$code"
}
trap failed ERR
trap 'echo "Interrupted; rerun with --resume"; exit 130' INT
trap 'echo "Terminated; rerun with --resume"; exit 143' TERM
cd "$ROOT"

source_ros() {
    set +u
    source /opt/ros/humble/setup.bash
    if [[ -f "$ROOT/install/setup.bash" ]]; then source "$ROOT/install/setup.bash"; fi
    set -u
}
check_1() {
    local status
    status=$(git submodule status --recursive)
    [[ -n "$status" && "$status" != *$'\n-'* && "${status:0:1}" != '-' ]] || return 1
    # Do not overwrite a locally checked-out submodule revision.
    [[ -f third_party/ros2_rm_robot/rm_driver/package.xml && -f third_party/OrbbecSDK_ROS2/orbbec_camera/package.xml && -f third_party/lerobot/pyproject.toml ]]
}
install_1() { git submodule update --init --recursive; }
# python3-tk provides the offline leader/follower slider GUI; numpy comes from system packages.
PACKAGES=(git git-lfs build-essential cmake pkg-config python3-venv python3-tk python3-colcon-common-extensions python3-rosdep python3-numpy python3-pil python3-yaml python3-serial ros-humble-rosbag2-py ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-storage-default-plugins ros-humble-moveit ros-humble-controller-interface ros-humble-controller-manager-msgs ros-humble-controller-manager libgflags-dev nlohmann-json3-dev ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-compressed-image-transport ros-humble-image-publisher ros-humble-camera-info-manager ros-humble-diagnostic-updater ros-humble-diagnostic-msgs ros-humble-statistics-msgs ros-humble-xacro ros-humble-backward-ros libdw-dev libssl-dev mesa-utils libgl1 libgoogle-glog-dev)
# These bundled U-Arm meshes are binary STL files stored in Git LFS.
check_leader_meshes() {
    /usr/bin/python3 - "$ROOT" <<'PYMESH'
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

root = Path(sys.argv[1]) / 'assets/robots/uarm_config2'
try:
    tree = ET.parse(root / 'uarm_config2_leader.urdf')
    meshes = {node.attrib['filename'] for node in tree.findall('.//mesh')}
    if not meshes:
        raise ValueError('URDF contains no meshes')
    for name in sorted(meshes):
        path = root / name
        data = path.read_bytes()
        if data.startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise ValueError(f'{path}: Git LFS pointer; model has not been downloaded')
        if len(data) < 84 or len(data) != 84 + 50 * struct.unpack_from('<I', data, 80)[0]:
            raise ValueError(f'{path}: incomplete binary STL')
except (OSError, ValueError, ET.ParseError) as exc:
    print(f'Leader mesh verification failed: {exc}', file=sys.stderr)
    sys.exit(1)
PYMESH
}
check_2() {
    local package
    for package in "${PACKAGES[@]}"; do
        [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null)" == 'install ok installed' ]] || return 1
    done
    check_leader_meshes
}
install_2() {
    sudo apt-get update
    sudo apt-get install -y "${PACKAGES[@]}"
    git lfs install --local
    git lfs pull --include="assets/robots/uarm_config2/meshes/*.stl" --exclude=""
    git lfs checkout assets/robots/uarm_config2/meshes/
    check_leader_meshes
}
sdk_source() {
    case "$(uname -m)" in
        x86_64) printf '%s\n' "$ROOT/third_party/ros2_rm_robot/rm_driver/lib/linux_x86_c++_v1.1.5/libapi_cpp.so" ;;
        aarch64) printf '%s\n' "$ROOT/third_party/ros2_rm_robot/rm_driver/lib/linux_arm64_c++_v1.1.5/libapi_cpp.so" ;;
        *) echo 'Unsupported CPU architecture' >&2; return 1 ;;
    esac
}
check_3() {
    local source
    source=$(sdk_source) || return 1
    cmp -s "$source" /usr/local/lib/libapi_cpp.so && cmp -s "$source" "$ROOT/third_party/ros2_rm_robot/rm_driver/lib/libapi_cpp.so" && /sbin/ldconfig -p | grep 'libapi_cpp.so' >/dev/null
}
install_3() {
    local source
    source=$(sdk_source)
    [[ -f "$source" ]]
    (cd "$ROOT/third_party/ros2_rm_robot/rm_driver/lib"; sudo bash lib_install.sh)
}
use_rosdistro_index() {
    if [[ -f "$STATE_DIR/rosdistro-index-url" ]]; then
        read -r ROSDISTRO_INDEX_URL < "$STATE_DIR/rosdistro-index-url"
        export ROSDISTRO_INDEX_URL
    fi
}
write_rosdep_sources() {
    local prefix="$1" tmp
    tmp=$(mktemp)
    cat > "$tmp" <<EOF
# os-specific listings first
yaml ${prefix}/rosdep/osx-homebrew.yaml osx

# generic
yaml ${prefix}/rosdep/base.yaml
yaml ${prefix}/rosdep/python.yaml
yaml ${prefix}/rosdep/ruby.yaml
gbpdistro ${prefix}/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml
EOF
    if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] && cmp -s "$tmp" /etc/ros/rosdep/sources.list.d/20-default.list; then
        rm -f "$tmp"
        return 0
    fi
    sudo mkdir -p /etc/ros/rosdep/sources.list.d
    sudo install -m 644 "$tmp" /etc/ros/rosdep/sources.list.d/20-default.list
    rm -f "$tmp"
}
prepare_rosdistro() {
    # rosdep init downloads its source list from GitHub and fails closed on timeout.
    local final="$STATE_DIR/rosdistro-index-v4.yaml"
    local -a names=("GitHub" "USTC" "TUNA" "TUNA-http")
    local -a urls=(
        "https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml"
        "https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml"
        "https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"
        "http://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"
    )
    local -a prefixes=(
        "https://raw.githubusercontent.com/ros/rosdistro/master"
        "https://mirrors.ustc.edu.cn/rosdistro"
        "https://mirrors.tuna.tsinghua.edu.cn/rosdistro"
        "http://mirrors.tuna.tsinghua.edu.cn/rosdistro"
    )
    local i
    for i in "${!names[@]}"; do
        printf 'Trying rosdistro index from %s...\n' "${names[$i]}"
        if /usr/bin/python3 - "${urls[$i]}" "${prefixes[$i]}" "$final" <<'PY'
import pathlib
import re
import sys
import urllib.request
from urllib.parse import urljoin
url, prefix, final = sys.argv[1:]
github = "https://raw.githubusercontent.com/ros/rosdistro/master"
request = urllib.request.Request(url, headers={"User-Agent": "xiazhi-setup"})
try:
    with urllib.request.urlopen(request, timeout=20) as response:
        text = response.read().decode("utf-8")
except Exception as exc:
    print(f"unreachable: {exc}", file=sys.stderr)
    raise SystemExit(1)
if "humble:" not in text or "distributions:" not in text:
    print("not a rosdistro index", file=sys.stderr)
    raise SystemExit(1)
if prefix != github:
    text = text.replace(github, prefix)
# The index lists each distro as a relative path. A file:// index URL would
# resolve humble/distribution.yaml inside .setup-state/, so make it absolute.
base_url = prefix.rstrip("/") + "/"

def absolute(value):
    value = value.strip().strip("'\"")
    if re.match(r"^[a-z][a-z0-9+.-]*://", value, re.I):
        return value
    return urljoin(base_url, value)

def replace_distribution(match):
    indent, body = match.group(1), match.group(2).strip()
    comment = ""
    if "#" in body:
        body, comment = body.split("#", 1)
        body = body.strip()
        comment = " #" + comment.rstrip()
    if body.startswith("["):
        inner = body[1:body.rfind("]")]
        parts = [absolute(part) for part in inner.split(",") if part.strip()]
        return f"{indent}[{', '.join(parts)}]{comment}"
    return f"{indent}{absolute(body)}{comment}"

text = re.sub(r"(?m)^(\s*distribution\s*:\s*)(\[[^\]]*\]|\S.*?)\s*$", replace_distribution, text)
pathlib.Path(final).write_text(text)
PY
        then
            printf 'Using rosdistro index from %s\n' "${names[$i]}"
            write_rosdep_sources "${prefixes[$i]}"
            printf 'file://%s\n' "$final" > "$STATE_DIR/rosdistro-index-url"
            export ROSDISTRO_INDEX_URL="file://${final}"
            return 0
        fi
    done
    echo 'Could not download a rosdistro index from GitHub, USTC, or TUNA' >&2
    return 1
}
check_4() {
    [[ -f /etc/ros/rosdep/sources.list.d/20-default.list && -f "$STATE_DIR/rosdep-updated" ]] || return 1
    use_rosdistro_index
    # rm_65_description is not a package; the URDF is in rm_description.
    # warehouse_ros_mongo has no Humble deb: MongoDB left Ubuntu 22.04.
    (source_ros; rosdep check --from-paths third_party/ros2_rm_robot third_party/OrbbecSDK_ROS2 --ignore-src --rosdistro humble --skip-keys "rm_65_description warehouse_ros_mongo" -t build -t buildtool -t build_export -t exec)
}
install_4() {
    prepare_rosdistro
    rosdep update --rosdistro humble
    touch "$STATE_DIR/rosdep-updated"
    (source_ros; rosdep install --from-paths third_party/ros2_rm_robot third_party/OrbbecSDK_ROS2 --ignore-src -y --rosdistro humble --skip-keys "rm_65_description warehouse_ros_mongo" -t build -t buildtool -t build_export -t exec)
}
CONFIG="$ROOT/third_party/ros2_rm_robot/rm_driver/config/rm_eco63_config.yaml"
configure_robot() {
    /usr/bin/python3 - "$CONFIG" "$ROBOT_IP" "$ROBOT_PORT" "$1" <<'PY'
import ipaddress
import pathlib
import re
import sys
path, requested_ip, requested_port, mode = sys.argv[1:]
p = pathlib.Path(path)
text = p.read_text()
if requested_port:
    if not requested_port.isdigit():
        raise ValueError('Invalid robot port')
    requested_port = str(int(requested_port))
for key, requested in [('arm_ip', requested_ip), ('tcp_port', requested_port)]:
    pattern = re.compile(r'^(\s*' + key + r':\s*)([^\n#]+)', re.M)
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError(f'Expected exactly one {key} in {path}')
    actual = matches[0].group(2).strip().strip('\"\'')
    value = requested or actual
    if key == 'arm_ip':
        ipaddress.IPv4Address(value)
    elif not value.isdigit() or not 1 <= int(value) <= 65535:
        raise ValueError('Invalid robot port')
    if requested and actual != requested:
        if mode == 'check':
            sys.exit(1)
        encoded = f'"{value}"' if key == 'arm_ip' else str(int(value))
        text = pattern.sub(lambda m: m.group(1) + encoded + ' ', text)
    print(f'{key}: {value}')
if mode == 'write' and text != p.read_text():
    p.write_text(text)
PY
}
check_5() { configure_robot check; }
install_5() { configure_robot write; }
source_hash() {
    # Include local source/config changes, not just submodule commits.
    { sha256sum "$ROOT/scripts/setup.sh"; find "$@" -type f ! -path '*/.git/*' -print0 | sort -z | xargs -0 sha256sum; } | sha256sum | cut -d' ' -f1
}
interface_hash() { source_hash third_party/ros2_rm_robot/rm_ros_interfaces; }
driver_hash() { source_hash third_party/ros2_rm_robot third_party/OrbbecSDK_ROS2; }
local_package() {
    local prefix
    prefix=$(ros2 pkg prefix "$1") || return 1
    [[ "$prefix" == "$ROOT/install/"* ]]
}
apply_drag_services() {
    # Official rm_ros_interfaces ships messages only. Drag collection calls these services.
    local dest="$ROOT/third_party/ros2_rm_robot/rm_ros_interfaces"
    local src="$ROOT/patches/rm_ros_interfaces/srv"
    [[ -d "$src" && -f "$dest/CMakeLists.txt" ]]
    mkdir -p "$dest/srv"
    cp -f "$src"/*.srv "$dest/srv/"
    /usr/bin/python3 - "$dest/CMakeLists.txt" <<'PY'
import pathlib
import sys
path = pathlib.Path(sys.argv[1])
text = path.read_text()
services = [
    "srv/CommandResult.srv",
    "srv/SaveDragTrajectory.srv",
    "srv/SetForceDragMode.srv",
    "srv/StartMultiDragTeach.srv",
]
anchor = "  ${msg_files}\n"
index = text.find(anchor)
if index < 0:
    raise SystemExit("rm_ros_interfaces CMakeLists.txt has no ${msg_files} entry")
addition = "".join(f"  {name}\n" for name in services if name not in text)
if addition:
    insert_at = index + len(anchor)
    path.write_text(text[:insert_at] + addition + text[insert_at:])
PY
}
check_6() {
    [[ -f "$STATE_DIR/interfaces.sha256" && "$(cat "$STATE_DIR/interfaces.sha256")" == "$(interface_hash)" ]] || return 1
    (source_ros; local_package rm_ros_interfaces && ros2 interface show rm_ros_interfaces/srv/StartMultiDragTeach >/dev/null)
}
install_6() {
    apply_drag_services
    (source_ros; colcon build --base-paths third_party/ros2_rm_robot third_party/OrbbecSDK_ROS2 --packages-select rm_ros_interfaces --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3)
    interface_hash > "$STATE_DIR/interfaces.sha256"
}
check_7() {
    [[ -f "$STATE_DIR/drivers.sha256" && "$(cat "$STATE_DIR/drivers.sha256")" == "$(driver_hash)" ]] || return 1
    (source_ros; local_package rm_driver && local_package rm_description && local_package orbbec_camera)
}
install_7() {
    (source_ros; colcon build --base-paths third_party/ros2_rm_robot third_party/OrbbecSDK_ROS2 --event-handlers console_direct+ --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3)
    driver_hash > "$STATE_DIR/drivers.sha256"
}
UDEV_DIR="$ROOT/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts"
check_8() { cmp -s "$UDEV_DIR/99-obsensor-libusb.rules" /etc/udev/rules.d/99-obsensor-libusb.rules && [[ ! -f /etc/udev/rules.d/99-obsensor-ros1-libusb.rules && -f "$STATE_DIR/udev-reloaded" ]]; }
install_8() { sudo bash "$UDEV_DIR/install_udev_rules.sh"; touch "$STATE_DIR/udev-reloaded"; }
verify_workspace() {
    check_2
    check_3
    check_6
    check_7
    check_8
    (source_ros
     ros2 interface show rm_ros_interfaces/srv/StartMultiDragTeach >/dev/null
     ros2 interface show rm_ros_interfaces/msg/Jointpos >/dev/null
     /usr/bin/python3 -c 'import rosbag2_py, rclpy, serial, numpy, PIL')
}
# Verification always runs, even when all earlier stages were skipped.
check_9() { return 1; }
install_9() { verify_workspace; }
check_10() {
    local python="$ROOT/.venv/bin/python"
    [[ -x "$python" && -f "$ROOT/.venv/pyvenv.cfg" ]] || return 1
    grep -qx 'include-system-site-packages = true' "$ROOT/.venv/pyvenv.cfg" || return 1
    [[ "$("$python" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" == 3.10 ]] || return 1
    "$python" -c 'import importlib.metadata, sys
try:
    version = importlib.metadata.version("rerun-sdk")
except importlib.metadata.PackageNotFoundError:
    sys.exit(1)
parts = version.split(".")
if parts[:2] != ["0", "33"]:
    sys.exit(1)
import numpy, matplotlib
from pathlib import Path
import mpl_toolkits
# Match the plotting script: avoid Ubuntu mpl_toolkits in this system-site venv.
mpl_toolkits.__path__ = [str(Path(matplotlib.__file__).resolve().parent.parent / "mpl_toolkits")]
from mpl_toolkits.mplot3d import Axes3D
numpy_version = tuple(map(int, numpy.__version__.split(".")[:2]))
matplotlib_version = tuple(map(int, matplotlib.__version__.split(".")[:2]))
sys.exit(0 if (2, 0) <= numpy_version < (2, 3) and (3, 9) <= matplotlib_version < (4, 0) else 1)' || return 1
}
install_10() {
    /usr/bin/python3 -m venv --system-site-packages "$ROOT/.venv"
    # Install a NumPy 2-compatible Matplotlib in the venv instead of using Ubuntu's build.
    "$ROOT/.venv/bin/python" -m pip install 'rerun-sdk>=0.33.0,<0.34.0' 'numpy>=2.0,<2.3' 'matplotlib>=3.9,<4'
}
for ((CURRENT_STEP=FROM_STEP;CURRENT_STEP<=TO_STEP;CURRENT_STEP++)); do
    checkpoint "$CURRENT_STEP"
    if "check_$CURRENT_STEP"; then
        printf '[%s/%s] ⏭ Already installed, skip %s\n' "$CURRENT_STEP" "$TOTAL" "${NAMES[$CURRENT_STEP]}"
    else
        printf '[%s/%s] Installing %s....\n' "$CURRENT_STEP" "$TOTAL" "${NAMES[$CURRENT_STEP]}"
        # Keep this call outside an if-condition: failures inside must trigger ERR.
        "install_$CURRENT_STEP"
        if ((CURRENT_STEP != 9)); then "check_$CURRENT_STEP"; fi
        printf '[%s/%s] ✅ Success install %s\n' "$CURRENT_STEP" "$TOTAL" "${NAMES[$CURRENT_STEP]}"
    fi
    checkpoint "$((CURRENT_STEP+1))"
done
printf 'Setup steps finished. Log: %s\n' "$LOG"
printf 'In your terminal: source /opt/ros/humble/setup.zsh; source %q/install/setup.zsh; source %q/.venv/bin/activate\n' "$ROOT" "$ROOT"
printf 'Reconnect cameras after installing udev rules. No hardware was connected by setup.\n'
