from subprocess import Popen, PIPE

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

commands = [
    "python pyboard.py -d /dev/ttyACM0 -f cp ",
]

for file in files_to_upload:
    proc = Popen("python pyboard.py -d /dev/ttyACM0 -f cat {}".format(file), stdout=PIPE, stderr=PIPE, shell=True)
    proc.wait()
    remote_files_contents = proc.communicate()[0]
    remote_files_contents = remote_files_contents[remote_files_contents.index(b"\n") + 1:].replace(b"\r\r\n", b"\r\n").strip()
    with open(file, "rb") as fp:
        local_file_contents = fp.read().strip()

    if remote_files_contents == local_file_contents:
        print("{} no update needed...".format(file))
    else:
        print("{} needs updating...".format(file))
        proc = Popen("python pyboard.py -d /dev/ttyACM0 -f cp {} :".format(file), stdout=PIPE, stderr=PIPE, shell=True)
        proc.wait()
        errors = proc.communicate()[1]
        if errors != b"":
            print(errors)
            exit(1)
        print("{} updated...".format(file))
