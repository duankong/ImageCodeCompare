import os


def addroot(input, output, source_path="/code/images/sourceimage/",
            target_path="/code/images/"):
    if os.path.splitext(input)[-1] == ".bmp" or os.path.splitext(input)[-1] == ".png":
        input = source_path + input
    else:
        input = target_path + input
    output = target_path + output
    return input, output


def showcmd(verbose, cmdline):
    if verbose > 0:
        print("[*] {}".format(cmdline))
