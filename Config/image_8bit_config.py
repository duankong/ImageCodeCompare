from collections import namedtuple

"""
----------------------------------------------
1 LOSSY || IMAGE || CODES || 8bit
----------------------------------------------
"""


def tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # 1
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
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '420'),
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '444'),
        # 3
        # CodecType('openjpeg', False, 10.0, 80.0, 0.05, '420'),
        # CodecType('openjpeg', False, 10.0, 80.0, 0.05, '444'),
        # # 4
        # CodecType('flif', False, 0, 100, 0.02, '420'),  # incomplete 4:2:0 chroma subsampled
        # CodecType('flif', False, 0, 100, 0.02, '444'),
        # # 5
        # CodecType('webp', False, 0, 100, 0.001, '420'),
        # # 6
        # CodecType('bpg', True, 0, 51, 0.01, '420'),
        # CodecType('bpg', True, 0, 51, 0.01, '444'),
        # # 7
        # CodecType('hevc', True, 10, 51, 1, '420'),
        # CodecType('hevc', True, 10, 51, 1, '444'),
        # 8
        # The codecs 'avif-mse' and 'avif-ssim' write out a "single-frame video" file in IVF format,
        # so technically not a proper AVIF. But they accomplish "full-range" YUV encoding similar to a "proper" AVIF.
        # CodecType('avif-mse', True, 1, 63, 1, '420'),
        # CodecType('avif-mse', True, 1, 63, 1, '444'),
        #
        # CodecType('avif-ssim', True, 1, 63, 1, '420'),
        # CodecType('avif-ssim', True, 1, 63, 1, '444'),
        # 9
        # Codecs starting with 'avifenc' prefix write out a proper AVIF; avifenc is a libavif-based tool.
        # https://github.com/AOMediaCodec/libavif
        # CodecType('avifenc-sp-0', True, 1, 62, 1, '420'),
        # CodecType('avifenc-sp-0', True, 1, 62, 1, '444'),
        #
        # CodecType('avifenc-sp-2', True, 1, 62, 1, '420'),
        # CodecType('avifenc-sp-2', True, 1, 62, 1, '444'),
        #
        # CodecType('avifenc-sp-4', True, 1, 62, 1, '420'),
        # CodecType('avifenc-sp-4', True, 1, 62, 1, '444'),
        #
        # CodecType('avifenc-sp-8', True, 1, 62, 1, '420'),
        # CodecType('avifenc-sp-8', True, 1, 62, 1, '444'),
    )
    return TUPLE_CODECS


"""
----------------------------------------------
2 LOSSLESS || IMAGE || CODES || 8bit
----------------------------------------------
"""


def lossless_tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', True, 0, 0, 0, '420'),
        CodecType('jpeg', True, 0, 0, 0, '444'),
        # # # 2
        CodecType('kakadu-mse', True, 0, 0, 0, '420'),
        CodecType('kakadu-mse', True, 0, 0, 0, '444'),
        #
        # CodecType('kakadu-visual', True, 0, 0, 0, '420'),
        # CodecType('kakadu-visual', True, 0, 0, 0, '444'),
        # # # 3
        # CodecType('openjpeg', True, 0, 0, 0, '420'),
        # CodecType('openjpeg', True, 0, 0, 0, '444'),
        # # # # 4
        # CodecType('flif', True, 0, 0, 0, '420'),
        # CodecType('flif', True, 0, 0, 0, '444'),
        # # # 5
        # CodecType('webp', True,0,0,0, '420'),
        # # # 6
        # CodecType('bpg', True, 0, 0, 0, '420'),
        # CodecType('bpg', True, 0, 0, 0, '444'),
        # # # # 7
        # CodecType('hevc', True, 0, 0, 0, '420'),
        # CodecType('hevc', True, 0, 0, 0, '444'),
        # # # # 8
        # CodecType('avif-mse', True, 0, 0, 0, '420'),
        # CodecType('avif-mse', True, 0, 0, 0, '444'),
        # # #
        # CodecType('avif-ssim', True, 0, 0, 0, '420'),
        # CodecType('avif-ssim', True, 0, 0, 0, '444'),
        # # # # 9
        # CodecType('avifenc-sp-0', True, 0, 0, 0, '420'),
        # CodecType('avifenc-sp-0', True, 0, 0, 0, '444'),
    )
    return TUPLE_CODECS
