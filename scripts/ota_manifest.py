"""Bind the OTA upload helper to the exact hybrid image built by PlatformIO."""
Import("env")

import hashlib
import json
from pathlib import Path
import re
import struct


def manifest(source, target, env):
    binary = Path(str(target[0]))
    header = Path(env.subst("$PROJECT_DIR")) / "include/gateway/OtaSecrets.hpp"
    match = re.search(r'#define HCR_OTA_PASSWORD_HASH "([0-9a-f]{32})"',
                      header.read_text() if header.exists() else "")
    data = binary.read_bytes()
    # ESP image header (24 B), first segment header (8 B), esp_app_desc_t.
    if len(data) < 288 or data[0] != 0xe9 or struct.unpack_from('<I',data,32)[0] != 0xabcd5432:
        raise RuntimeError('Missing ESP application descriptor; cannot identify OTA image')
    record = {"environment": env.subst("$PIOENV"), "bytes": len(data),
              "sha256": hashlib.sha256(data).hexdigest(),
              "elf_sha256": data[176:208].hex(),
              "ota_auth_hash": match.group(1) if match else None}
    binary.with_suffix(".ota.json").write_text(json.dumps(record, indent=2) + "\n")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", manifest)
