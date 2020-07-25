import os


def addroot(input, output, source_path="/home/duankong/test_image/sourceimage/",
            target_path="/home/duankong/test_image/"):
    if os.path.splitext(input)[-1] == ".bmp" or os.path.splitext(input)[-1] == ".png":
        input = source_path + input
    else:
        input = target_path + input
    output = target_path + output
    return input, output


def showcmd(verbose, cmdline):
    if verbose > 0:
        print("[*] {}".format(cmdline))
