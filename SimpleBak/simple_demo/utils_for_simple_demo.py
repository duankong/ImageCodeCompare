import os
import skimage.io as io
from skimage import metrics

def addroot(input, output, source_path="/code/images/sourceimage/",
            target_path="/code/images/"):
    if os.path.splitext(input)[-1] == ".bmp" or os.path.splitext(input)[-1] == ".png":
        input = source_path + input
    else:
        input = target_path + input
    output = target_path + output
    return input, output


def showcmd(verbose, cmdline):
    if verbose > 0:
        print("[*] {}".format(cmdline))


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
