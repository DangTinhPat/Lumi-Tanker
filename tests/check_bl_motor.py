#!/usr/bin/env python3
"""Build and exercise the deployed motor core/codec with simulated driver replies."""
from pathlib import Path
import argparse
import subprocess
from check_hybrid_gateway import check_interop
import tempfile
ROOT = Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runtime",type=Path)
    args=ap.parse_args()
    with tempfile.TemporaryDirectory(prefix='hcr-bl-motor-') as directory:
        binary = Path(directory)/'motor-tests'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
                        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                        '-I'+str(ROOT/'include'),
                        str(ROOT/'src/BlMotor.cpp'),
                        str(ROOT/'src/HybridGatewayCore.cpp'),
                        str(ROOT/'tests/bl_motor_tests.cpp'),
                        '-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
        if args.runtime:
            check_interop(binary,args.runtime.resolve(),absent_cases=(False,))
            print("PASS: LF Core + actual BL motor backend/codec, two mock driver buses, 2700 cycles")

if __name__=='__main__':
    main()
