# ====================================
# define the image code class
# ====================================
import os
from utils import addroot, showcmd


class JPEG:
    def __init__(self):
        self.cmdpath = "/tools/jpeg-9d/"
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


class JPEG2000:
    def __init__(self):
        self.cmdpath = "/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602/"
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


class WebP:
    def __init__(self):
        self.cmdpath = "/tools/libwebp-1.1.0-linux-x86-64/bin/"
        self.code = "cwebp"
        self.decode = "dwebp"
        self.quality = ' -q 75'
        self.silence = ' -quiet'

    def code_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switches = self.silence
        cmdline = self.cmdpath + self.code + switches + " " + input + " -o " + output
        showcmd(verbose, cmdline)
        os.system(cmdline)

    def decode_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switchs = self.silence
        cmdline = self.cmdpath + self.decode + switchs + " " + input + " -o " + output
        showcmd(verbose, cmdline)
        os.system(cmdline)


class BPG:
    def __init__(self):
        self.cmdpath = "/home/duankong/picture/bpg/libbpg-0.9.8/"
        self.code = "bpgenc"
        self.decode = "bpgdec"

    def code_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switches = ""
        cmdline = self.cmdpath + self.code + switches + " " + input + " -o " + output
        showcmd(verbose, cmdline)
        os.system(cmdline)

    def decode_image(self, input, output, verbose=0):
        input, output = addroot(input, output)
        switchs = ""
        cmdline = self.cmdpath + self.decode + switchs + " " + input + " -o " + output
        showcmd(verbose, cmdline)
        os.system(cmdline)


if __name__ == '__main__':
    bmpfile = 'testout.bmp'
    jp2file = 'duankong_demo.jp2'
    jpgfile = "duankong_demo.jpg"
    decodefile = "decode.bmp"
    decodejpg = "decode_jpg.bmp"
    # JPEG
    jpeg = JPEG()
    jpeg.code_image(input=bmpfile, output=jpgfile)
    jpeg.decode_image(input=jpgfile, output=decodejpg)
    # JPEG2000
    # code = JPEG2000()
    # code.code_image(input=bmpfile,output=jp2file)
    # code.decode_image(input=jp2file, output=decodefile, verbose=1)
