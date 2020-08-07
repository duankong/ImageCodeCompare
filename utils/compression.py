import os
import ntpath

from skimage import measure, io
from collections import namedtuple

from .utils_common import get_filename_with_temp_folder


def tuple_codes():
    CodecType = namedtuple('CodecType', ['name', 'inverse', 'param_start', 'param_end', 'ab_tol', 'subsampling'])
    TUPLE_CODECS = (

        CodecType('jpeg', False, 5, 95, 1, '444u'),
        #
        CodecType('jpegxt', False, 5, 95, 1, '444u'),
        #
        CodecType('webp', False, 0, 100, 0.1, '420'),
        #
        CodecType('kakadu', False, 0.01, 3.0, 0.03, '420'),
        #
        CodecType('bpg', True, 0, 51, 0.02, '420'),
        #
        CodecType('flif', False, 0, 100, 0.02, '420'),

    )
    return TUPLE_CODECS


def compute_metrics(ref_image, dist_image, temp_folder):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    ref_image_path = get_filename_with_temp_folder(temp_folder, ntpath.basename(ref_image))
    dis_image_path = get_filename_with_temp_folder(temp_folder, ntpath.basename(dist_image))
    log_path = get_filename_with_temp_folder(temp_folder, 'stats.log')
    source = io.imread(ref_image_path)
    encode = io.imread(dis_image_path)
    mse_value = measure.compare_mse(source, encode)
    if mse_value == 0:
        psnr_value = float("inf")
    else:
        psnr_value = measure.compare_psnr(source, encode, data_range=255)

    ssim_value = measure.compare_ssim(source, encode)
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
