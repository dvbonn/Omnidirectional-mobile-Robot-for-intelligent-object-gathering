"""
Setup Script: Install llama.cpp and download the Qwen2.5-VL model
================================================================
This script will:
  1. Check prerequisites (git, cmake, python)
  2. Download the Qwen2.5-VL-3B-Instruct model (GGUF 4-bit)
  3. Set up llama.cpp (detect a pre-built binary or build from source)
  4. Generate the server launch scripts

Requirements: Python 3.8+, Git
"""

import io
import json
import platform
import subprocess
import sys
from pathlib import Path

# Fix Windows console encoding
if sys.platform == "win32":
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

# Paths
SCRIPT_DIR   = Path(__file__).parent
PROJECT_DIR  = SCRIPT_DIR.parent
CONFIG_PATH  = SCRIPT_DIR / "config.json"
MODELS_DIR   = SCRIPT_DIR / "models"
LLAMA_DIR    = PROJECT_DIR / "llama.cpp"

# Load config
with open(CONFIG_PATH, encoding="utf-8") as f:
    CONFIG = json.load(f)

IS_WINDOWS = platform.system() == "Windows"

# Helpers
def header(title: str):
    print(f"\n{'-' * 60}")
    print(f"  {title}")
    print(f"{'-' * 60}\n")

def ok(msg):  print(f"  [OK] {msg}")
def err(msg): print(f"  [ERR] {msg}")
def warn(msg):print(f"  [WARN] {msg}")
def info(msg):print(f"  [INFO] {msg}")
def step(n, msg): print(f"  [{n}] {msg}")


# 1. Prerequisites
def check_prerequisites() -> bool:
    header("CHECK PREREQUISITES")

    tools = {"python": "python --version", "pip": "pip --version", "git": "git --version"}
    all_ok = True

    for name, cmd in tools.items():
        try:
            r = subprocess.run(cmd.split(), capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                ok(f"{name}: {(r.stdout or r.stderr).strip()}")
            else:
                err(f"{name}: not found"); all_ok = False
        except FileNotFoundError:
            err(f"{name}: not found"); all_ok = False

    # CMake - optional (only needed when building from source)
    try:
        r = subprocess.run(["cmake", "--version"], capture_output=True, text=True, timeout=10)
        if r.returncode == 0:
            ok(f"cmake: {r.stdout.splitlines()[0]}")
        else:
            warn("cmake: missing - will prefer a pre-built binary")
    except FileNotFoundError:
        warn("cmake: missing - will prefer a pre-built binary")

    return all_ok


# 2. Download Model
def download_model() -> bool:
    header("DOWNLOAD QWEN2.5-VL-3B-INSTRUCT MODEL (GGUF Q4_K_M)")

    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    repo       = CONFIG["vlm"]["model_repo"]
    model_file = CONFIG["vlm"]["model_file"]
    mmproj_file= CONFIG["vlm"]["mmproj_file"]
    model_path = MODELS_DIR / model_file
    mmproj_path= MODELS_DIR / mmproj_file

    if model_path.exists() and mmproj_path.exists():
        ok(f"Model    : {model_path.name}")
        ok(f"MMProj   : {mmproj_path.name}")
        return True

    info(f"Repository : {repo}")
    info(f"Model file : {model_file}  (~2-3 GB)")
    info(f"MMProj file: {mmproj_file}")
    info(f"Saved to   : {MODELS_DIR}\n")

    try:
        from huggingface_hub import hf_hub_download

        for i, (fname, fpath) in enumerate([
            (model_file, model_path),
            (mmproj_file, mmproj_path),
        ], start=1):
            if not fpath.exists():
                step(i, f"Downloading {fname}...")
                hf_hub_download(
                    repo_id=repo, filename=fname,
                    local_dir=str(MODELS_DIR), local_dir_use_symlinks=False,
                )
                ok(f"Downloaded: {fname}")
            else:
                step(i, f"Already present: {fname}")

        return True

    except ImportError:
        err("Missing the huggingface-hub library. Installing...")
        subprocess.run([sys.executable, "-m", "pip", "install", "huggingface-hub"])
        warn("Please re-run the script after installation completes.")
        return False

    except Exception as e:
        err(f"Model download error: {e}")
        print("\n  Download manually with:")
        print(f"     huggingface-cli download {repo} {model_file} --local-dir {MODELS_DIR}")
        print(f"     huggingface-cli download {repo} {mmproj_file} --local-dir {MODELS_DIR}")
        return False


# 3. Setup llama.cpp
def _find_prebuilt_server() -> Path | None:
    """Find llama-server.exe in common locations."""
    candidates = [
        LLAMA_DIR / ("llama-server.exe" if IS_WINDOWS else "llama-server"),
        LLAMA_DIR / "bin" / ("llama-server.exe" if IS_WINDOWS else "llama-server"),
        LLAMA_DIR / "build" / "bin" / "Release" / "llama-server.exe",
        LLAMA_DIR / "build" / "bin" / "llama-server",
    ]
    for p in candidates:
        if p.exists():
            return p
    return None


def _clone_or_update():
    if LLAMA_DIR.exists():
        ok(f"Repo already present: {LLAMA_DIR}")
        step(1, "Updating llama.cpp (git pull)...")
        subprocess.run(["git", "pull"], cwd=str(LLAMA_DIR), capture_output=True)
    else:
        step(1, "Cloning llama.cpp...")
        r = subprocess.run(
            ["git", "clone", "https://github.com/ggml-org/llama.cpp.git", str(LLAMA_DIR)],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            err(f"Clone failed: {r.stderr[:200]}")
            return False
        ok(f"Cloned: {LLAMA_DIR}")
    return True


def _build_from_source() -> bool:
    step(2, "Building llama.cpp from source (CMake)...")
    build_dir = LLAMA_DIR / "build"
    build_dir.mkdir(exist_ok=True)

    cmake_cfg = ["cmake", ".."]
    if IS_WINDOWS:
        cmake_cfg += ["-G", "MinGW Makefiles"]

    try:
        r = subprocess.run(cmake_cfg, cwd=str(build_dir),
                           capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            warn(f"CMake configure error:\n     {r.stderr[:300]}")
            return False

        r = subprocess.run(["cmake", "--build", ".", "--config", "Release", "-j"],
                           cwd=str(build_dir),
                           capture_output=True, text=True, timeout=600)
        if r.returncode == 0:
            ok("Build succeeded!")
            return True
        else:
            warn(f"Build failed:\n     {r.stderr[:300]}")
            return False

    except FileNotFoundError:
        warn("cmake not found, skipping the build step.")
        return False
    except subprocess.TimeoutExpired:
        warn("Build timeout (> 10 minutes).")
        return False


def setup_llama_cpp() -> bool:
    header("SET UP LLAMA.CPP")

    # Priority 1: an existing pre-built binary
    server = _find_prebuilt_server()
    if server:
        ok(f"Found pre-built binary: {server.relative_to(PROJECT_DIR)}")
        r = subprocess.run([str(server), "--version"], capture_output=True, text=True)
        version_line = (r.stdout + r.stderr).strip().splitlines()
        for line in version_line:
            if "version" in line.lower() or "built with" in line.lower():
                info(line)
        return True

    # Priority 2: clone + build from source
    info("No pre-built binary, trying to build from source...")
    if not _clone_or_update():
        return False

    if _build_from_source():
        return True

    # Priority 3: manual instructions
    release_url = "https://github.com/ggml-org/llama.cpp/releases"
    err("Could not build automatically.")
    print(f"\n  Please download a pre-built binary from:\n     {release_url}")
    print(f"     Download: llama-<version>-bin-win-avx2.zip (Windows)")
    print(f"     Extract into: {LLAMA_DIR}\n")
    return False


# 4. Generate launch scripts
def create_launch_scripts():
    header("GENERATE SERVER LAUNCH SCRIPTS")

    cfg    = CONFIG["vlm"]
    srv    = CONFIG["llama_cpp_server"]
    mfile  = cfg["model_file"]
    mproj  = cfg["mmproj_file"]
    host   = srv["host"]
    port   = srv["port"]
    ctx    = cfg["context_size"]
    ngl    = cfg["gpu_layers"]
    threads= cfg["threads"]

    model_rel  = f"layer2_brain/models/{mfile}"
    mmproj_rel = f"layer2_brain/models/{mproj}"

    # Auto-detect the exe path
    server = _find_prebuilt_server()
    if server:
        exe_win  = str(server.relative_to(PROJECT_DIR))
        exe_bash = exe_win.replace("\\", "/")
    else:
        exe_win  = "llama.cpp\\llama-server.exe"
        exe_bash = "llama.cpp/llama-server"

    # Windows .bat
    if IS_WINDOWS:
        bat = f"""\
@echo off
REM Run from the project root (this script lives in scripts/)
cd /d "%~dp0.."
echo ================================================
echo   LLAMA.CPP SERVER  --  Qwen2.5-VL-3B-Instruct
echo ================================================
echo.

if not exist "{model_rel}" (
    echo [ERR] Not found: {model_rel}
    echo       Run: python layer2_brain/setup_vlm.py
    pause & exit /b 1
)

set SERVER={exe_win}
if not exist "%SERVER%" (
    where llama-server >nul 2>&1
    if errorlevel 1 (
        echo [ERR] llama-server.exe not found
        echo       Download from: https://github.com/ggml-org/llama.cpp/releases
        pause & exit /b 1
    )
    set SERVER=llama-server
)

echo Server : %SERVER%
echo Model  : {model_rel}
echo MMProj : {mmproj_rel}
echo Host   : {host}:{port}
echo Context: {ctx} tokens  ^|  GPU layers: {ngl}  ^|  Threads: {threads}
echo.

%SERVER% ^
    --model "{model_rel}" ^
    --mmproj "{mmproj_rel}" ^
    --host {host} ^
    --port {port} ^
    --ctx-size {ctx} ^
    --n-gpu-layers {ngl} ^
    --threads {threads} ^
    --chat-template chatml

pause
"""
        bat_path = PROJECT_DIR / "scripts" / "start_vlm_server.bat"
        bat_path.parent.mkdir(exist_ok=True)
        bat_path.write_text(bat, encoding="utf-8")
        ok(f"Windows batch : {bat_path.name}")

    # Shell .sh
    sh = f"""\
#!/bin/bash
# Run from the project root (this script lives in scripts/)
cd "$(dirname "$0")/.." || exit 1
echo "================================================"
echo "  LLAMA.CPP SERVER  --  Qwen2.5-VL-3B-Instruct"
echo "================================================"
echo

[ ! -f "{model_rel}" ] && {{
    echo "[ERR] Not found: {model_rel}"
    echo "      Run: python layer2_brain/setup_vlm.py"
    exit 1
}}

SERVER=""
for candidate in \\
    "llama.cpp/llama-server" \\
    "llama.cpp/bin/llama-server" \\
    "llama.cpp/build/bin/llama-server" \\
    "llama.cpp/build/bin/Release/llama-server"; do
    if [ -f "$candidate" ]; then
        SERVER="$candidate"
        break
    fi
done
[ -z "$SERVER" ] && SERVER=$(which llama-server 2>/dev/null)
[ -z "$SERVER" ] && {{
    echo "[ERR] llama-server not found"
    echo "      Download from: https://github.com/ggml-org/llama.cpp/releases"
    echo "      Extract and place at: llama.cpp/llama-server"
    exit 1
}}
chmod +x "$SERVER" 2>/dev/null

echo "Server : $SERVER"
echo "Model  : {model_rel}"
echo "Host   : {host}:{port}"
echo "Context: {ctx} tokens | GPU layers: {ngl} | Threads: {threads}"
echo

"$SERVER" \\
    --model "{model_rel}" \\
    --mmproj "{mmproj_rel}" \\
    --host {host} \\
    --port {port} \\
    --ctx-size {ctx} \\
    --n-gpu-layers {ngl} \\
    --threads {threads} \\
    --chat-template chatml
"""
    sh_path = PROJECT_DIR / "scripts" / "start_vlm_server.sh"
    sh_path.parent.mkdir(exist_ok=True)
    sh_path.write_text(sh, encoding="utf-8", newline="\n")
    ok(f"Shell script  : {sh_path.name}")


# Main
def main():
    header(f"SETUP VLM  |  {platform.system()} {platform.machine()}")
    print(f"  Python  : {sys.version.split()[0]}")
    print(f"  Project : {PROJECT_DIR}\n")

    prereq_ok = check_prerequisites()
    if not prereq_ok:
        warn("Some tools are missing, continuing anyway...\n")

    model_ok = download_model()
    llama_ok = setup_llama_cpp()
    create_launch_scripts()

    # Summary
    header("RESULT")
    print(f"  Model download  : {'OK' if model_ok else 'Needs manual download'}")
    print(f"  llama.cpp       : {'OK' if llama_ok else 'Needs manual install'}")
    print()
    print("  USAGE:")
    print("  ------")
    launcher = "scripts\\start_vlm_server.bat" if IS_WINDOWS else "./scripts/start_vlm_server.sh"
    print(f"  1. Start the VLM server   :  {launcher}")
    print(f"  2. Start the Brain API    :  python layer2_brain/brain_server.py")
    print(f"  3. Start the Control node :  python layer3_control/control_node.py")
    print(f"  4. Start the Vision node  :  python layer1_vision/vision_node.py")

    if not model_ok:
        print("\n  MANUAL MODEL DOWNLOAD:")
        repo = CONFIG["vlm"]["model_repo"]
        print(f"     pip install huggingface-hub")
        print(f"     huggingface-cli download {repo} {CONFIG['vlm']['model_file']} --local-dir layer2_brain/models")
        print(f"     huggingface-cli download {repo} {CONFIG['vlm']['mmproj_file']} --local-dir layer2_brain/models")

    if not llama_ok:
        print("\n  MANUAL LLAMA.CPP INSTALL:")
        print(f"     Download: https://github.com/ggml-org/llama.cpp/releases")
        print(f"     Extract into: {LLAMA_DIR}")

    if model_ok and llama_ok:
        print("\n  SETUP COMPLETE - CAN RUN OFFLINE")
        print("     The system is ready to run without internet.")
        print("     (HF_HUB_OFFLINE=1 is enabled automatically when brain_server starts)")


if __name__ == "__main__":
    main()
