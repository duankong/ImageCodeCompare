import os


def addroot(input, output, source_path="/home/duankong/test_image/", target_path="/home/duankong/test_image/"):
    input = source_path + input
    output = target_path + output
    return input, output


def showcmd(verbose, cmdline):
    if verbose > 0:
        print("[*] {}".format(cmdline))


class JPEG():
    def __init__(self):
        self.cmdpath = "/home/duankong/picture/jpeg/jpeg-9d/"
        self.code = "cjpeg"
        self.decode = "djpeg"
        self.dct = " -dct int"
        self.outfile = " -outfile"
        self.quality = " -quality 75"
        self.bmp = " -bmp"

    def code_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switches = self.quality + self.dct + self.outfile
        cmdline = self.cmdpath + self.code + switches + " " + output + " " + input
        showcmd(verbose, cmdline)
        os.system(cmdline)

    def decode_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switchs = self.dct + self.bmp + self.outfile
        cmdline = self.cmdpath + self.decode + switchs + " " + output + " " + input
        showcmd(verbose, cmdline)
        os.system(cmdline)


class JPEG2000():
    def __init__(self):
        self.cmdpath = "/home/duankong/picture/jpeg2000/KDU805_Demo_Apps_for_Linux-x86-64_200602/"
        self.code = "kdu_compress"
        self.decode = "kdu_expand"
        self.Creversible = " Creversible=yes"
        self.rate = ' -rate -'
        self.silence = ' -quiet'

    def code_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switches = self.Creversible + self.rate + self.silence
        cmdline = self.cmdpath + self.code + " -i " + input + " -o " + output + switches
        showcmd(verbose, cmdline)
        os.system(cmdline)

    def decode_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switchs = self.silence
        cmdline = self.cmdpath + self.decode + " -i " + input + " -o " + output + switchs
        showcmd(verbose, cmdline)
        os.system(cmdline)


if __name__ == '__main__':
    bmpfile = 'testout.bmp'
    jp2file = 'duankong_demo.jp2'
    jpgfile = "duankong_demo.jpg"
    decodefile = "decode.bmp"
    decodejpg="decode_jpg.bmp"
    # JPEG
    jpeg = JPEG()
    jpeg.code_image(input=bmpfile, output=jpgfile)
    jpeg.decode_image(input=jpgfile,output=decodejpg)
    # JPEG2000
    # code = JPEG2000()
    # code.code_image(input=bmpfile,output=jp2file)
    # code.decode_image(input=jp2file, output=decodefile, verbose=1)
