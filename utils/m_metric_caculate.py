from collections import defaultdict
from numpy import mean
import json
from .u_utils_common import get_filename_with_temp_folder
from .u_ffmpeg_format import get_pixel_format
from .u_run_cmd import run_program
from .u_folder_build import remove_files


def compute_vmaf(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        use the ffmpeg libvmaf filter to compute vmaf, vif, ssim, and ms_ssim.
    """
    pixel_format = get_pixel_format(subsampling, depth)

    log_path = get_filename_with_temp_folder(temp_folder, 'stats_vmaf.json')

    cmd = ['ffmpeg', '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', dist_image,
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', ref_image,
           '-lavfi', 'libvmaf=psnr=true:ssim=true:ms_ssim=true:log_fmt=json:log_path=' + log_path, '-f', 'null', '-']

    run_program(LOGGER, cmd)
    vmaf_dict = dict()
    try:
        vmaf_log = json.load(open(log_path))
    except:
        LOGGER.error("{}".format(log_path))
    else:
        vmaf_dict["vmaf"] = vmaf_log["frames"][0]["metrics"]["vmaf"]
        vmaf_dict["vif"] = vmaf_log["frames"][0]["metrics"]["vif_scale0"]
        vmaf_dict["ssim"] = vmaf_log["frames"][0]["metrics"]["ssim"]
        vmaf_dict["ms_ssim"] = vmaf_log["frames"][0]["metrics"]["ms_ssim"]
        vmaf_dict["adm2"] = vmaf_log["frames"][0]["metrics"]["adm2"]
    return vmaf_dict


def compute_psnr(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        use the ffmpeg psnr filter to compute psnr and mse for each channel.
    """
    pixel_format = get_pixel_format(subsampling, depth)
    log_path = get_filename_with_temp_folder(temp_folder, 'stats_psnr.log')
    cmd = ['ffmpeg',
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', dist_image,
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', ref_image,
           '-lavfi', 'psnr=stats_file=' + log_path, '-f', 'null', '-']
    run_program(LOGGER, cmd)
    psnr_dict = get_mean_psnr(log_path)
    return psnr_dict


def compute_metrics(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    vmaf = compute_vmaf(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling)
    psnr = compute_psnr(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling)
    stats = vmaf.copy()
    stats.update(psnr)
    remove_files(ref_image)
    remove_files(dist_image)
    return stats


def get_mean_psnr(log_path):
    psnr_dict = defaultdict(list)
    f = open(log_path)
    line = f.readline()
    while line:
        for stat in line.rstrip().split(" "):
            key, value = stat.split(":")
            psnr_dict[key].append(float(value))
        line = f.readline()
    f.close()

    psnr_avg = dict()
    for key, value in psnr_dict.items():
        psnr_avg[key] = mean(value)

    return psnr_avg


if __name__ == '__main__':
    pass
