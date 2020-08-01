import os
from Cmdline import JPEG, JPEG2000, WebP, BPG

if __name__ == '__main__':
    bmpfile = 'flower.bmp'
    pngfile = 'net.png'

    jp2file = 'duankong_jp2.jp2'
    jpgfile = "duankong_jpeg.jpg"
    webpfile = "duankong_webp.webp"
    bpgfile = "duankong-bpg.bpg"

    decodejp2 = "decode_jp2.bmp"
    decodejpg = "decode_jpg.bmp"
    decodewebp = "decode_webp.png"
    decodebpg = "decode_bpg.png"

    verbose = 1
    # JPEG
    jpeg = JPEG()
    jpeg.code_image(input=bmpfile, output=jpgfile, verbose=verbose)
    jpeg.decode_image(input=jpgfile, output=decodejpg, verbose=verbose)
    # JPEG2000
    code = JPEG2000()
    code.code_image(input=bmpfile, output=jp2file, verbose=verbose)
    code.decode_image(input=jp2file, output=decodejp2, verbose=verbose)
    # WebP
    webp = WebP()
    webp.code_image(input=pngfile, output=webpfile, verbose=verbose)
    webp.decode_image(input=webpfile, output=decodewebp, verbose=verbose)
    # BPG
    bpg = BPG()
    bpg.code_image(input=pngfile, output=bpgfile, verbose=verbose)
    bpg.decode_image(input=bpgfile, output=decodebpg, verbose=verbose)

    print("[ ] Done !")
