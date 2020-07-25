def addroot(input, output, source_path="/home/duankong/test_image/", target_path="/home/duankong/test_image/"):
    input = source_path + input
    output = target_path + output
    return input, output


def showcmd(verbose, cmdline):
    if verbose > 0:
        print("[*] {}".format(cmdline))
