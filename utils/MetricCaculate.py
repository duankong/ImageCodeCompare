import json
from .UtilsCommon import get_filename_with_temp_folder, get_pixel_format_for_metric_computation, \
    get_pixel_format_for_metric_computation_16bit
from .RunCmd import run_program


def compute_vmaf(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        use the ffmpeg libvmaf filter to compute vmaf, vif, ssim, and ms_ssim.
    """
    pixel_format = get_pixel_format_for_metric_computation(
        subsampling) if depth == 8 else get_pixel_format_for_metric_computation_16bit(subsampling)

    log_path = get_filename_with_temp_folder(temp_folder, 'stats_vmaf.json')

    cmd = ['ffmpeg', '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', dist_image,
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', ref_image,
           '-lavfi', 'libvmaf=psnr=true:ssim=true:ms_ssim=true:log_fmt=json:log_path=' + log_path, '-f', 'null', '-']

    run_program(LOGGER, cmd)

    vmaf_log = json.load(open(log_path))

    vmaf_dict = dict()
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

    pixel_format = get_pixel_format_for_metric_computation(
        subsampling) if depth == 8 else get_pixel_format_for_metric_computation_16bit(subsampling)

    log_path = get_filename_with_temp_folder(temp_folder, 'stats_psnr.log')
    cmd = ['ffmpeg',
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', dist_image,
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', ref_image,
           '-lavfi', 'psnr=stats_file=' + log_path, '-f', 'null', '-']

    run_program(LOGGER, cmd)

    psnr_dict = dict()
    psnr_log = open(log_path).read()
    for stat in psnr_log.rstrip().split(" "):
        key, value = stat.split(":")
        if key != "n":
            psnr_dict[key] = float(value)
    return psnr_dict


def compute_metrics(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    vmaf = compute_vmaf(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling)
    psnr = compute_psnr(LOGGER, ref_image, dist_image, width, height, depth, temp_folder, subsampling)
    stats = vmaf.copy()
    stats.update(psnr)
    return stats


if __name__ == '__main__':
    pass
