import os
import ntpath

from skimage import measure, io
from collections import namedtuple

from .utils_common import get_filename_with_temp_folder


def tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (
        # CodecType('jpeg', False, 5, 100, 1, '420'),
        # CodecType('jpeg', False, 5, 100, 1, '444'),
        # CodecType('jpeg', False, 5, 100, 1, '444u'),

        # CodecType('jpeg-mse', False, 5, 100, 1, '420'),
        # CodecType('jpeg-mse', False, 5, 100, 1, '444'),
        # CodecType('jpeg-mse', False, 5, 100, 1, '444u'),

        # CodecType('jpeg-ms-ssim', False, 5, 100, 1, '420'),
        # CodecType('jpeg-ms-ssim', False, 5, 100, 1, '444'),
        # CodecType('jpeg-ms-ssim', False, 5, 100, 1, '444u'),

        # CodecType('jpeg-im', False, 5, 100, 1, '420'),
        # CodecType('jpeg-im', False, 5, 100, 1, '444'),
        # CodecType('jpeg-im', False, 5, 100, 1, '444u'),

        # CodecType('jpeg-hvs-psnr', False, 5, 100, 1, '420'),
        # CodecType('jpeg-hvs-psnr', False, 5, 100, 1, '444'),
        # CodecType('jpeg-hvs-psnr', False, 5, 100, 1, '444u'),

        # # webp can only encode 420
        # CodecType('webp', False, 5, 100, 1, '420'),
        # CodecType('webp', False, 5, 100, 1, '444u'),

        CodecType('kakadu-mse', False, 0.01, 3.0, 0.03, '420'),
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.03, '444'),
        # CodecType('kakadu-mse', False, 0.01, 3.0, 0.03, '444u'),

        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.03, '420'),
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.03, '444'),
        # CodecType('kakadu-visual', False, 0.01, 3.0, 0.03, '444u'),

        # CodecType('openjpeg', False, 30.0, 60.0, 0.05, '420'),
        # CodecType('openjpeg', False, 30.0, 60.0, 0.05, '444'),
        # CodecType('openjpeg', False, 30.0, 60.0, 0.05, '444u'),

        # CodecType('hevc', True, 10, 51, 1, '420'),
        # CodecType('hevc', True, 10, 51, 1, '444'),
        # CodecType('hevc', True, 10, 51, 1, '444u'),

        # CodecType('avif-mse', True, 8, 63, 1, '420'),
        # CodecType('avif-mse', True, 8, 63, 1, '444'),
        # CodecType('avif-mse', True, 8, 63, 1, '444u'),

        # CodecType('avif-ssim', True, 8, 63, 1, '420'),
        # CodecType('avif-ssim', True, 8, 63, 1, '444'),
        # CodecType('avif-ssim', True, 8, 63, 1, '444u'),

    )
    return  TUPLE_CODECS


def compute_metrics(ref_image, dist_image, width, height, temp_folder):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    ref_image_path = os.getcwd() + '/' + get_filename_with_temp_folder(temp_folder, ntpath.basename(ref_image))
    dis_image_path = os.getcwd() + '/' + get_filename_with_temp_folder(temp_folder, ntpath.basename(dist_image))
    log_path = get_filename_with_temp_folder(temp_folder, 'stats.log')

    source = io.imread(ref_image_path)
    encode = io.imread(dis_image_path)
    psnr_value = measure.compare_psnr(source, encode, data_range=255)
    mse_value = measure.compare_mse(source, encode)
    ssim_value = measure.compare_ssim(source, encode, multichannel=True)
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
