from collections import namedtuple

"""
----------------------------------------------
1 LOSSY || IMAGE || CODES || 8bit
----------------------------------------------
"""


def tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', False, 1, 100, 0.5, '420'),
        CodecType('jpeg', False, 1, 100, 0.5, '444'),
        #
        # CodecType('jpeg-mse', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-mse', False, 1, 100, 0.5, '444'),
        # #
        # CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '444'),
        # #
        # CodecType('jpeg-im', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-im', False, 1, 100, 0.5, '444'),
        # #
        # CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '444'),
        # 2
        CodecType('kakadu-mse', False, 0.01, 3.0, 0.01, '420'),
        CodecType('kakadu-mse', False, 0.01, 3.0, 0.01, '444'),

        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '420'),
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '444'),
        # 3
        CodecType('openjpeg', False, 10.0, 80.0, 0.05, '420'),
        CodecType('openjpeg', False, 10.0, 80.0, 0.05, '444'),
        # 4
        CodecType('flif', False, 0, 100, 0.02, '420'),  # incomplete 4:2:0 chroma subsampled
        CodecType('flif', False, 0, 100, 0.02, '444'),
        # 5
        CodecType('webp', False, 0, 100, 0.001, '420'),
        # 6
        CodecType('bpg', True, 0, 51, 0.01, '420'),
        CodecType('bpg', True, 0, 51, 0.01, '444'),
        # 7
        CodecType('hevc', True, 10, 51, 1, '420'),
        CodecType('hevc', True, 10, 51, 1, '444'),
        # 8
        CodecType('avif-mse', True, 8, 63, 1, '420'),
        CodecType('avif-mse', True, 8, 63, 1, '444'),

        CodecType('avif-ssim', True, 8, 63, 1, '420'),
        CodecType('avif-ssim', True, 8, 63, 1, '444'),
    )
    return TUPLE_CODECS


"""
----------------------------------------------
2 LOSSLESS || IMAGE || CODES || 8bit
----------------------------------------------
"""


def lossless_tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'param_lossy', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', 100, '420'),
        CodecType('jpeg', 100, '444'),
        # # 2
        CodecType('kakadu-mse', 3.0, '420'),
        CodecType('kakadu-mse', 3.0, '444'),

        CodecType('kakadu-visual', 3.0, '420'),
        CodecType('kakadu-visual', 3.0, '444'),
        # # 3
        CodecType('openjpeg', 1, '420'),
        CodecType('openjpeg', 1, '444'),
        # # 4
        CodecType('flif', 100, '420'),  # incomplete 4:2:0 chroma subsampled
        CodecType('flif', 100, '444'),
        # # 5
        CodecType('webp', 100, '420'),
        # # 6
        CodecType('bpg', 0, '420'),
        CodecType('bpg', 0, '444'),
        # # 7
        CodecType('hevc', 0, '420'),
        CodecType('hevc', 0, '444'),
        # # 8
        CodecType('avif-mse', 63, '420'),
        CodecType('avif-mse', 63, '444'),

        CodecType('avif-ssim', 63, '420'),
        CodecType('avif-ssim', 63, '444'),
    )
    return TUPLE_CODECS


"""
----------------------------------------------
3 LOSSY || IMAGE || CODES || 16bit
----------------------------------------------
"""


def tuple_codes_high_dynamic_range():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', False, 1, 100, 0.5, '420'),
        CodecType('jpeg', False, 1, 100, 0.5, '444'),
        #
        # CodecType('jpeg-mse', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-mse', False, 1, 100, 0.5, '444'),
        # #
        # CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '444'),
        # #
        # CodecType('jpeg-im', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-im', False, 1, 100, 0.5, '444'),
        # #
        # CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '444'),
        # 2
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.01, '420'),
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.01, '444'),
        #
        # # CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '420'),
        # # CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '444'),
        # ## 3
        # CodecType('openjpeg', False, 10.0, 80.0, 0.05, '420'),
        # CodecType('openjpeg', False, 10.0, 80.0, 0.05, '444'),
        # ## 4
        # CodecType('flif', False, 0, 100, 0.02, '420'),  # incomplete 4:2:0 chroma subsampled
        # CodecType('flif', False, 0, 100, 0.02, '444'),
        # ## 5
        # CodecType('webp', False, 0, 100, 0.001, '420'),
        # ## 6
        # CodecType('bpg', True, 0, 51, 0.01, '420'),
        # CodecType('bpg', True, 0, 51, 0.01, '444'),
        # ## 7
        # CodecType('hevc', True, 10, 51, 1, '420'),
        # CodecType('hevc', True, 10, 51, 1, '444'),
        # ## 8
        # CodecType('avif-mse', True, 8, 63, 1, '420'),
        # CodecType('avif-mse', True, 8, 63, 1, '444'),
        #
        # CodecType('avif-ssim', True, 8, 63, 1, '420'),
        # CodecType('avif-ssim', True, 8, 63, 1, '444'),
    )
    return TUPLE_CODECS


"""
----------------------------------------------
4 LOSSLESS HighDynamicRange || IMAGE || CODES || 16bit
----------------------------------------------
"""


def lossless_tuple_codes_high_dynamic_range():
    CodecType = namedtuple('CodecType', ['name', 'param_lossy', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', 100, '420'),
        CodecType('jpeg', 100, '444'),
        # # 2
        CodecType('kakadu-mse', 3.0, '420'),
        CodecType('kakadu-mse', 3.0, '444'),

        CodecType('kakadu-visual', 3.0, '420'),
        CodecType('kakadu-visual', 3.0, '444'),
        # # 3
        CodecType('openjpeg', 1, '420'),
        CodecType('openjpeg', 1, '444'),
        # # 4
        CodecType('flif', 100, '420'),
        CodecType('flif', 100, '444'),
        # # 5
        # CodecType('webp', 100, '420'),# not support 16bit
        # CodecType('webp', 100, '444'),# not support 444
        # # 6
        # CodecType('bpg', 0, '420'),// 8 to 12 not support 16bit
        # CodecType('bpg', 0, '444'),// 8 to 12 not support 16bit
        # # 7
        CodecType('hevc', 0, '420'),
        CodecType('hevc', 0, '444'),
        # # 8
        CodecType('avif-mse', 63, '420'),
        CodecType('avif-mse', 63, '444'),

        CodecType('avif-ssim', 63, '420'),
        CodecType('avif-ssim', 63, '444'),
    )
    return TUPLE_CODECS


class format_adress:
    def __init__(self):
        self.jpeg = '/tools/jpeg'
        self.webp = '/tools/libwebp-1.1.0-linux-x86-64/bin'
        self.kakadu = '/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602'
        self.heif = '/tools/libheif-1.7.0/build/bin'
        self.hevc = '/usr/bin'
        self.highbithevc = '/tools/HM-16.20+SCM-8.8/bin'
        self.webp = '/tools/libwebp-1.1.0-linux-x86-64/bin'
        self.bpg = '/tools/libbpg-master'


if __name__ == '__main__':
    pass
