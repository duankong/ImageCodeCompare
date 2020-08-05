import os
import ntpath
from skimage import measure, io
from .utils_common import get_filename_with_temp_folder


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
