from collections import namedtuple

"""
----------------------------------------------
3 LOSSY || IMAGE || CODES || 16bit
----------------------------------------------
"""


def tuple_codes_high_dynamic_range():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # ## 1 || MAX 83.02 86.03
        # CodecType('jpeg', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg', False, 1, 100, 0.5, '444'),
        #
        # CodecType('jpeg-mse', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-mse', False, 1, 100, 0.5, '444'),

        CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '420'),
        CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '444'),
        #
        # CodecType('jpeg-im', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-im', False, 1, 100, 0.5, '444'),
        #
        # CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '420'),
        # CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '444'),
        # # ## 2 || MAX 84.34 87.35
        # CodecType('kakadu-mse', False, 0.01, 16.0, 0.01, '420'),
        # CodecType('kakadu-mse', False, 0.01, 16.0, 0.01, '444'),
        #
        CodecType('kakadu-visual', False, 0.01, 16.0, 0.01, '420'),
        CodecType('kakadu-visual', False, 0.01, 16.0, 0.01, '444'),
        # # # ## 3 || MAX 46.72 49.73
        CodecType('openjpeg', False, 10.0, 200, 0.05, '420'),
        CodecType('openjpeg', False, 10.0, 160, 0.05, '444'),
        # ## 4 || MAX 84.28 87.29
        CodecType('flif', False, 0, 100, 0.02, '420'),  # incomplete 4:2:0 chroma subsampled
        CodecType('flif', False, 0, 100, 0.02, '444'),
        # ## 5
        # CodecType('webp', False, 0, 100, 0.001, '420'),# not support 16bit
        # CodecType('webp', False, 0, 100, 0.001, '444'),# not support 444
        # ## 6
        # CodecType('bpg', True, 0, 51, 0.01, '420'),# 8 to 12 not support 16bit
        # CodecType('bpg', True, 0, 51, 0.01, '444'),# 8 to 12 not support 16bit
        ## 7 || MAX 65.37 68.08
        CodecType('hevc', True, 0, 51, 0.2, '420'),
        CodecType('hevc', True, 0, 51, 0.2, '444'),
        # ## 8 || MAX 56.32 59.83
        # CodecType('avif-mse', True, 1, 63, 1, '420'),# 8 to 12 not support 16bit
        # CodecType('avif-mse', True, 1, 63, 1, '444'),# 8 to 12 not support 16bit
        # #
        # CodecType('avif-ssim', True, 1, 63, 1, '420'),# 8 to 12 not support 16bit
        # CodecType('avif-ssim', True, 1, 63, 1, '444'),# 8 to 12 not support 16bit
        # ## 9
        # CodecType('avifenc-sp-0', True, 1, 62, 1, '420'),# 8 to 12 not support 16bit
        # CodecType('avifenc-sp-0', True, 1, 62, 1, '444'),# 8 to 12 not support 16bit
    )
    return TUPLE_CODECS


"""
----------------------------------------------
4 LOSSLESS HighDynamicRange || IMAGE || CODES || 16bit
----------------------------------------------
"""


def lossless_tuple_codes_high_dynamic_range():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # 1
        CodecType('jpeg', True, 0, 0, 0, '420'),
        CodecType('jpeg', True, 0, 0, 0, '444'),
        # # 2
        # CodecType('kakadu-mse', True, 0, 0, 0, '420'),
        # CodecType('kakadu-mse', True, 0, 0, 0, '444'),
        #
        CodecType('kakadu-visual', True, 0, 0, 0, '420'),
        CodecType('kakadu-visual', True, 0, 0, 0, '444'),
        # # # 3
        CodecType('openjpeg', True, 0, 0, 0, '420'),
        CodecType('openjpeg', True, 0, 0, 0, '444'),
        # # # 4
        CodecType('flif', True, 0, 0, 0, '420'),
        CodecType('flif', True, 0, 0, 0, '444'),
        # # # 5
        # CodecType('webp', True, 0, 0, 0, '420'),# not support 16bit
        # CodecType('webp', True, 0, 0, 0, '444'),# not support 444
        # # # 6
        # CodecType('bpg', True, 0, 0, 0,'420'),# 8 to 12 not support 16bit
        # CodecType('bpg', True, 0, 0, 0,'444'),# 8 to 12 not support 16bit
        # # # 7
        CodecType('hevc', True, 0, 0, 0, '420'),
        CodecType('hevc', True, 0, 0, 0, '444'),
        # # # 8
        # CodecType('avif-mse', True, 0, 0, 0, '420'),# 8 to 12 not support 16bit
        # CodecType('avif-mse', True, 0, 0, 0, '444'),# 8 to 12 not support 16bit
        #
        # CodecType('avif-ssim', True, 0, 0, 0, '420'),# 8 to 12 not support 16bit
        # CodecType('avif-ssim', True, 0, 0, 0, '444'),# 8 to 12 not support 16bit
        # # # 9
        # CodecType('avifenc-sp-0', True, 0, 0, 0, '420'),# 8 to 12 not support 16bit
        # CodecType('avifenc-sp-0', True, 0, 0, 0, '444'),# 8 to 12 not support 16bit
    )
    return TUPLE_CODECS
