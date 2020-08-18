import ntpath

from skimage import io, metrics
from collections import namedtuple

from .utils_common import get_filename_with_temp_folder


def tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # # 1
        # CodecType('jpeg', False, 5, 100, 1, '420'),
        CodecType('jpeg', False, 5, 100, 1, '444'),
        # CodecType('jpeg', False, 5, 100, 1, '444u'),
        #
        # CodecType('jpeg-mse', False, 5, 100, 1, '420'),
        # CodecType('jpeg-mse', False, 5, 100, 1, '444'),
        # CodecType('jpeg-mse', False, 5, 100, 1, '444u'),
        #
        # CodecType('jpeg-ms-ssim', False, 5, 100, 1, '420'),
        # CodecType('jpeg-ms-ssim', False, 5, 100, 1, '444'),
        # CodecType('jpeg-ms-ssim', False, 5, 100, 1, '444u'),
        # #
        # CodecType('jpeg-im', False, 5, 100, 1, '420'),
        # CodecType('jpeg-im', False, 5, 100, 1, '444'),
        # CodecType('jpeg-im', False, 5, 100, 1, '444u'),
        # #
        # CodecType('jpeg-hvs-psnr', False, 5, 100, 1, '420'),
        # CodecType('jpeg-hvs-psnr', False, 5, 100, 1, '444'),
        # CodecType('jpeg-hvs-psnr', False, 5, 100, 1, '444u'),
        # # 2
        # CodecType('webp', False, 0, 100, 0.1, '420'),
        # CodecType('webp', False, 5, 100, 1, '444u'),
        # # 3
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.03, '420'),
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.03, '444'),
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.03, '444u'),
        #
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.03, '420'),
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.03, '444'),
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.03, '444u'),
        # #
        # CodecType('openjpeg', False, 30.0, 60.0, 0.05, '420'),
        # CodecType('openjpeg', False, 30.0, 60.0, 0.05, '444'),
        # CodecType('openjpeg', False, 30.0, 60.0, 0.05, '444u'),
        # # 4
        # CodecType('avif-mse', True, 8, 63, 1, '420'),
        # CodecType('avif-mse', True, 8, 63, 1, '444'),
        # CodecType('avif-mse', True, 8, 63, 1, '444u'),
        #
        # CodecType('avif-ssim', True, 8, 63, 1, '420'),
        # CodecType('avif-ssim', True, 8, 63, 1, '444'),
        # CodecType('avif-ssim', True, 8, 63, 1, '444u'),
        # # 5
        # CodecType('bpg', True, 0, 51, 0.03, '420'),
        # CodecType('bpg', True, 0, 51, 0.03, '444'),# BUG
        # CodecType('bpg', True, 0, 51, 0.03, '444u'),
        # # 6
        # CodecType('flif', False, 0, 100, 0.02, '420'),
        # CodecType('flif', False, 0, 100, 0.02, '444'),
        # CodecType('flif', False, 0, 100, 0.02, '444u'),
        # # 7
        # CodecType('heif', False, 0, 100, 0.02, '444'),


    )
    return TUPLE_CODECS


def compute_metrics_skcikit(ref_image, dist_image, temp_folder):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    ref_image_path = get_filename_with_temp_folder(temp_folder, ntpath.basename(ref_image))
    dis_image_path = get_filename_with_temp_folder(temp_folder, ntpath.basename(dist_image))
    log_path = get_filename_with_temp_folder(temp_folder, 'stats.log')
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
