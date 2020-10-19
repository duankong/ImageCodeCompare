from collections import namedtuple

"""
----------------------------------------------
5 VIDEO COMPRESS || lossy || 有损  8bit
----------------------------------------------
"""


def video_tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # ## 1 HEVC
        # CodecType('hevc', True, 0, 51, 0.5, '420'),
        # CodecType('hevc', True, 0, 51, 0.5, '444'),
        # ## 2 VVC
        # CodecType('vvc', True, 0, 51, 0.5, '420'),
        # CodecType('vvc', True, 0, 51, 0.5, '444'),
        # ## 3 AVS3
        # CodecType('avs3', True, 0, 63, 1, '420'),# 只支持 8bit 有损 420
        # ## 4 AOM
        CodecType('aom-mse', True, 1, 63, 1, '420'),# 8 to 12
        CodecType('aom-mse', True, 1, 63, 1, '444'),# 8 to 12
        #
        # CodecType('aom-ssim', True, 1, 63, 1, '420'),# 8 to 12
        # CodecType('aom-ssim', True, 1, 63, 1, '444'),# 8 to 12
        # ## 5 RAV1E
        # CodecType('rav1e-Psychovisual', True, 0, 255, 1, '420'),
        # CodecType('rav1e-Psychovisual', True, 0, 255, 1, '444'),
        #
        # CodecType('rav1e-Psnr', True, 0, 255, 1, '420'),
        # CodecType('rav1e-Psnr', True, 0, 255, 1, '444'),
        # ## 6 SVT-AV1
        # CodecType('svt-av1', True, 0, 63, 0.5, '420'), # 只支持 8bit 有损 420
        # ## 7
        # CodecType('vp8-psnr', True, 4, 59, 0.5, '420'), # 只支持 8bit 有损 420
        # CodecType('vp8-ssim', True, 4, 59, 0.5, '420'),
        # # ## 8
        # CodecType('vp9-psnr', True, 4, 59, 0.5, '420'),
        # CodecType('vp9-psnr', True, 4, 59, 0.5, '444'),
        # CodecType('vp9-ssim', True, 4, 59, 0.5, '420'),
        # CodecType('vp9-ssim', True, 4, 59, 0.5, '444'),
        # ## 9 SVT-VP9
        # 只支持 8bit 有损 420
        # CodecType('svt-vp9-visually', True, 0, 63, 0.5, '420'),
        # CodecType('svt-vp9-PSNR-SSIM', True, 0, 63, 0.5, '420'),
        # CodecType('svt-vp9-VMAF', True, 0, 63, 0.5, '420'),

    )
    return TUPLE_CODECS


"""
----------------------------------------------
5 VIDEO COMPRESS || lossless || 无损  8bit
----------------------------------------------
"""


def video_tuple_lossless_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # ## 1 HEVC
        # CodecType('hevc', True, 0, 51, 0.5, '420'),
        # CodecType('hevc', True, 0, 51, 0.5, '444'),
        # # ## 2 VVC
        # CodecType('vvc', True, 0, 51, 0.5, '420'),
        # CodecType('vvc', True, 0, 51, 0.5, '444'),
        #
        # # ## 4 AOM
        CodecType('aom-mse', True, 1, 63, 1, '420'),# 8 to 12
        CodecType('aom-mse', True, 1, 63, 1, '444'),# 8 to 12
        # #
        # CodecType('aom-ssim', True, 1, 63, 1, '420'),# 8 to 12
        # CodecType('aom-ssim', True, 1, 63, 1, '444'),# 8 to 12
        #
        # # ## 8
        # CodecType('vp9-psnr', True, 4, 59, 0.5, '420'),
        # CodecType('vp9-psnr', True, 4, 59, 0.5, '444'),
        # CodecType('vp9-ssim', True, 4, 59, 0.5, '420'),
        # CodecType('vp9-ssim', True, 4, 59, 0.5, '444'),

    )
    return TUPLE_CODECS
