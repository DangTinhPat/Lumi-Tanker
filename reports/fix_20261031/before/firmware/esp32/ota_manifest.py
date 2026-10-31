"""Bind the OTA upload helper to the exact hybrid image built by PlatformIO."""
Import("env")

import hashlib
import json
from pathlib import Path
import re


def manifest(source, target, env):
    binary = Path(str(target[0]))
    header = Path(env.subst("$PROJECT_DIR")) / "firmware/esp32/include/tap_hw/OtaSecrets.hpp"
    match = re.search(r'#define HCR_OTA_PASSWORD_HASH "([0-9a-f]{32})"',
                      header.read_text() if header.exists() else "")
    record = {"environment": env.subst("$PIOENV"), "bytes": binary.stat().st_size,
              "sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
              "ota_auth_hash": match.group(1) if match else None}
    binary.with_suffix(".ota.json").write_text(json.dumps(record, indent=2) + "\n")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", manifest)
