Import("env")
import os
import json

lib_dir = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "FreeRTOS-Kernel"
)

lib_json_path = os.path.join(lib_dir, "library.json")

if os.path.isdir(lib_dir):
    config = {
        "name": "FreeRTOS-Kernel",
        "version": "10.5.1",
        "build": {
            "flags": [
                "-Iinclude",
                "-Iportable/GCC/ARM_CM3"
            ],
            "srcDir": ".",
            "srcFilter": [
                "+<*.c>",
                "-<portable/>",
                "+<portable/GCC/ARM_CM3/port.c>",
                "+<portable/MemMang/heap_4.c>"
            ]
        }
    }
    with open(lib_json_path, "w") as f:
        json.dump(config, f, indent=2)