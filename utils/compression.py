from skimage import io, metrics
from collections import namedtuple


def tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', False, 1, 100, 0.5, '420'),
        CodecType('jpeg', False, 1, 100, 0.5, '444'),
        #
        CodecType('jpeg-mse', False, 1, 100, 0.5, '420'),
        CodecType('jpeg-mse', False, 1, 100, 0.5, '444'),
        #
        CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '420'),
        CodecType('jpeg-ms-ssim', False, 1, 100, 0.5, '444'),
        # #
        CodecType('jpeg-im', False, 1, 100, 0.5, '420'),
        CodecType('jpeg-im', False, 1, 100, 0.5, '444'),
        #
        CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '420'),
        CodecType('jpeg-hvs-psnr', False, 1, 100, 0.5, '444'),
        # 2
        CodecType('webp', False, 0, 100, 0.001, '420'),
        # 3
        CodecType('kakadu-mse', False, 0.01, 3.0, 0.01, '420'),
        CodecType('kakadu-mse', False, 0.01, 3.0, 0.01, '444'),

        CodecType('kakadu-visual', False, 0.01, 3.0,0.01, '420'),
        CodecType('kakadu-visual', False, 0.01, 3.0, 0.01, '444'),

        CodecType('openjpeg', False, 10.0, 80.0, 0.05, '420'),# BUG
        CodecType('openjpeg', False, 10.0, 80.0, 0.05, '444'),
        # 4
        CodecType('avif-mse', True, 8, 63, 1, '420'),
        CodecType('avif-mse', True, 8, 63, 1, '444'),

        CodecType('avif-ssim', True, 8, 63, 1, '420'),
        CodecType('avif-ssim', True, 8, 63, 1, '444'),
        # 5
        CodecType('bpg', True, 0, 51, 0.01, '420'),
        CodecType('bpg', True, 0, 51, 0.01, '444'),
        # 6
        CodecType('flif', False, 0, 100, 0.02, '420'),# incomplete 4:2:0 chroma subsampled
        CodecType('flif', False, 0, 100, 0.02, '444'),
        #  7
        CodecType('hevc', True, 10, 51, 1, '420'),
        CodecType('hevc', True, 10, 51, 1, '444'),

    )
    return TUPLE_CODECS


def lossy_tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'param_lossy', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', 100, '420'),
        CodecType('jpeg', 100, '444'),
        # # # 2
        CodecType('webp', 100, '420'),
        # # 3
        CodecType('kakadu-mse', 3.0, '420'),
        CodecType('kakadu-mse', 3.0, '444'),

        # CodecType('kakadu-visual', 3.0, '420'),
        # CodecType('kakadu-visual', 3.0, '444'),
        # # #
        CodecType('openjpeg', 1, '420'),  # BUG
        CodecType('openjpeg', 1, '444'),
        # 4
        CodecType('avif-mse', 63, '420'),
        CodecType('avif-mse', 63, '444'),

        # CodecType('avif-ssim', 63, '420'),
        # CodecType('avif-ssim', 63, '444'),
        # 5
        CodecType('bpg', 0, '420'),
        CodecType('bpg', 0, '444'),
        # 6
        CodecType('flif', 100, '420'),  # incomplete 4:2:0 chroma subsampled
        CodecType('flif', 100, '444'),
        # 7
        CodecType('hevc', 0, '420'),
        CodecType('hevc', 0, '444'),

    )
    return TUPLE_CODECS

def lossy_tuple_codes_high_dynamic_range():
    CodecType = namedtuple('CodecType', ['name', 'param_lossy', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        CodecType('jpeg', 100, '420'),
        CodecType('jpeg', 100, '444'),
        # # # 2
        CodecType('webp', 100, '420'),
        # # 3
        CodecType('kakadu-mse', 3.0, '420'),
        CodecType('kakadu-mse', 3.0, '444'),

        # CodecType('kakadu-visual', 3.0, '420'),
        # CodecType('kakadu-visual', 3.0, '444'),
        # # #
        CodecType('openjpeg', 1, '420'),  # BUG
        CodecType('openjpeg', 1, '444'),
        # 4
        CodecType('avif-mse', 63, '420'),
        CodecType('avif-mse', 63, '444'),

        # CodecType('avif-ssim', 63, '420'),
        # CodecType('avif-ssim', 63, '444'),
        # 5
        CodecType('bpg', 0, '420'),
        CodecType('bpg', 0, '444'),
        # 6
        CodecType('flif', 100, '420'),  # incomplete 4:2:0 chroma subsampled
        CodecType('flif', 100, '444'),
        # 7
        CodecType('hevc', 0, '420'),
        CodecType('hevc', 0, '444'),

    )
    return TUPLE_CODECS

class format_adress:
    def __init__(self):
        self.jpeg = '/tools/jpeg'
        self.webp = '/tools/libwebp-1.1.0-linux-x86-64/bin'
        self.kakadu = '/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602'
        self.heif = '/tools/libheif-1.7.0/build/bin'
        self.hevc = '/usr/bin'
        self.webp = '/tools/libwebp-1.1.0-linux-x86-64/bin'
        self.bpg='/tools/libbpg-master'


def write_log(ref_image_path, dis_image_path, log_path):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    source = io.imread(ref_image_path)
    encode = io.imread(dis_image_path)
    mse_value = metrics.mean_squared_error(source, encode)
    if mse_value == 0:
        psnr_value = float("inf")
    else:
        psnr_value = metrics.peak_signal_noise_ratio(source, encode, data_range=255)

    ssim_value = metrics.structural_similarity(source, encode, multichannel=True)
    stats = dict()
    stats['psnr'] = psnr_value
    stats['mse'] = mse_value
    stats['ssim'] = ssim_value

    with open(log_path, 'w') as fob:
        fob.write("[*] ref_image={}\n".format(ref_image_path))
        fob.write("[*] dis_image={}\n".format(dis_image_path))
        fob.write("[*] MSE={}\n".format(mse_value))
        fob.write("[*] PSNR={}\n".format(psnr_value))
        fob.write("[*] SSIM={}\n".format(ssim_value))
    return stats


if __name__ == '__main__':
    pass
