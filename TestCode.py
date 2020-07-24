import os
from CmdLine import JPEG
if __name__ == '__main__':
    print("hello world")
    jpeg=JPEG()
    inputfile="/home/duankong/图片/testout.bmp"
    outputfile="/home/duankong/图片/duankong3.jpg"
    jpeg.code_image(inputfile,outputfile)
