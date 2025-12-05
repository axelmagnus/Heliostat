#!/usr/bin/env python3
import os
import sys
import time
import glob
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET = ROOT / "heliostat.ino"
FQBN = "esp32:esp32:adafruit_feather_esp32s3_reversetft"

def run(cmd, cwd=ROOT, check=True):
    print(f"\n$ {cmd}")
    result = subprocess.run(cmd, cwd=str(cwd), shell=True)
    if check and result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")
    return result.returncode

def detect_port():
    ports = glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/cu.usbserial*')
    return ports[0] if ports else None

def compile_and_upload():
    # Compile
    rc = run(f"arduino-cli compile --fqbn {FQBN} heliostat.ino")
    if rc != 0:
        return rc
    # Detect port
    port = detect_port()
    if not port:
        print("No serial port found under /dev/cu.usbmodem* or /dev/cu.usbserial*")
        return 1
    # Upload
    return run(f"arduino-cli upload -p {port} --fqbn {FQBN} heliostat.ino")

def main():
    if not TARGET.exists():
        print(f"File not found: {TARGET}")
        return 1
    print(f"Watching for saves: {TARGET}")
    last_mtime = TARGET.stat().st_mtime
    try:
        while True:
            try:
                mtime = TARGET.stat().st_mtime
            except FileNotFoundError:
                time.sleep(0.5)
                continue
            if mtime != last_mtime:
                # Debounce short bursts of writes
                time.sleep(0.25)
                last_mtime = mtime
                # Prompt user
                try:
                    ans = input("heliostat.ino saved. Compile and upload now? [y/N]: ").strip().lower()
                except EOFError:
                    ans = "n"
                if ans == "y" or ans == "yes":
                    compile_and_upload()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nWatcher stopped.")
        return 0

if __name__ == "__main__":
    sys.exit(main())
