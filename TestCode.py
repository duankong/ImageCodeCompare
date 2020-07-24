import os
from CmdLine import JPEG, JPEG2000

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

    print("[ ] Doen !")
