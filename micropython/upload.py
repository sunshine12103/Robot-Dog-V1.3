import time
from subprocess import Popen, PIPE

from pyboard import Pyboard

files_to_upload = [
    "boot.py",
    "crawl.py",
    "lcd_api.py",
    "main.py",
    "pca9685.py",
    "profiler.py",
    "pyb_i2c_lcd.py",
    "rc.py",
    "state.py",
]

port = "/dev/ttyACM0"

for i, file in enumerate(files_to_upload):
    proc = Popen("python pyboard.py -d {} -f cat {}".format(port, file), stdout=PIPE, stderr=PIPE, shell=True)
    proc.wait()
    proc_com = proc.communicate()
    errors = proc_com[1]
    if errors != b"":
        print(errors)
        exit(1)
    remote_files_contents = proc_com[0]
    remote_files_contents = remote_files_contents[remote_files_contents.index(b"\n") + 1:].replace(b"\r\n", b"\n")
    with open(file, "rb") as fp:
        local_file_contents = fp.read()

    if remote_files_contents == local_file_contents:
        print("{}/{}:{}\n  no update needed...".format(i + 1, len(files_to_upload), file))
    else:
        print("{}\n  needs updating...".format(file))
        proc = Popen("python pyboard.py -d {} -f cp {} :".format(port, file), stdout=PIPE, stderr=PIPE, shell=True)
        proc.wait()
        errors = proc.communicate()[1]
        if errors != b"":
            print(errors)
            exit(1)
        print("  updated...")

output_path = "/tmp/spot_micro.txt"
print("Dumping serial output to {}...".format(output_path))
print("Use <ctrl> + <c> to exit")
pyb = Pyboard(port)
pyb.serial.write(b"\x04")  # soft reset
pyb.serial.timeout = 0.2
try:
    with open(output_path, "wb", buffering=64) as file_pointer:
        while True:
            try:
                file_pointer.write(pyb.serial.read(64))
            except TimeoutError:
                pass
except KeyboardInterrupt:
    pyb.close()
