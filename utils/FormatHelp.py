import ntpath
import glob
import os
from .RunCmd import run_program, my_exec
from .CompressConfig import format_adress
from .UtilsCommon import get_filename_with_temp_folder, get_pixel_format_for_encoding, \
    get_pixel_format_for_encoding_16bit

FROM = format_adress()


def get_dimensions(LOGGER, image):
    """ given a source image, return dimensions and bit-depth
    """
    dimension_cmd = ["identify", '-format', '%w,%h,%z', image]
    res = run_program(LOGGER, dimension_cmd)
    res_split = res.split('\n')
    width, height, depth = res_split[len(res_split) - 1].split(
        ',')  # assuming last line has the goodies; warnings, if any, appear before
    return width, height, depth


def kakadu_encode_helper(LOGGER, image, subsampling, temp_folder, source_yuv, param, codec):
    cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
    my_exec(LOGGER, cmd)

    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mj2')
    # kakadu derives width, height, sub-sampling from file name so avoid confusion with folder name
    cmd = ['{}/kdu_v_compress'.format(FROM.kakadu), '-quiet', '-i',
           ntpath.basename(source_yuv), '-o', ntpath.basename(encoded_file), '-precise', '-rate', '-,3',
           'Creversible=yes', '-tolerance',
           '0']
    if codec == 'kakadu-mse':
        cmd[-2:-2] = ['-no_weights']
    run_program(LOGGER, cmd, cwd=temp_folder)

    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded.yuv')

    for filePath in glob.glob(get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded_*.yuv')):
        try:
            os.remove(filePath)
        except:
            LOGGER.error("Error while deleting file : " + filePath)

    cmd = ['{}/kdu_v_expand'.format(FROM.kakadu), '-quiet', '-i', encoded_file,
           '-o', decoded_yuv]
    my_exec(LOGGER, cmd)

    decoded_yuv_files = glob.glob(get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded_*.yuv'))
    assert len(decoded_yuv_files) == 1

    decoded_yuv = decoded_yuv_files[0]

    return encoded_file, decoded_yuv, source_yuv


def kakadu_encode_helper_16bit(LOGGER, image, subsampling, temp_folder, source_yuv, param, codec):
    cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding_16bit(subsampling), source_yuv]
    my_exec(LOGGER, cmd)

    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mj2')
    # kakadu derives width, height, sub-sampling from file name so avoid confusion with folder name
    cmd = ['{}/kdu_v_compress'.format(FROM.kakadu), '-quiet', '-i',
           ntpath.basename(source_yuv), '-o', ntpath.basename(encoded_file), '-precise', '-rate', '-,3',
           'Creversible=yes', '-tolerance',
           '0']
    if codec == 'kakadu-mse':
        cmd[-2:-2] = ['-no_weights']
    run_program(LOGGER, cmd, cwd=temp_folder)

    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded.yuv')

    for filePath in glob.glob(get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded_*.yuv')):
        try:
            os.remove(filePath)
        except:
            LOGGER.error("Error while deleting file : " + filePath)

    cmd = ['{}/kdu_v_expand'.format(FROM.kakadu), '-quiet', '-i', encoded_file,
           '-o', decoded_yuv]
    my_exec(LOGGER, cmd)

    decoded_yuv_files = glob.glob(get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded_*.yuv'))
    assert len(decoded_yuv_files) == 1

    decoded_yuv = decoded_yuv_files[0]

    return encoded_file, decoded_yuv, source_yuv


def jpeg_encode_helper(LOGGER, codec, cmd, encoded_file, temp_folder):
    if codec == 'jpeg-mse':
        cmd[1:1] = ['-qt', '1']
    elif codec == 'jpeg-ms-ssim':
        cmd[1:1] = ['-qt', '2']
    elif codec == 'jpeg-im':
        cmd[1:1] = ['-qt', '3']
    elif codec == 'jpeg-hvs-psnr':
        cmd[1:1] = ['-qt', '4']
    run_program(LOGGER, cmd)

    cmd = ['/tools/jpeg/jpeg', encoded_file, get_filename_with_temp_folder(temp_folder, 'decoded.ppm')]
    run_program(LOGGER, cmd)


if __name__ == '__main__':
    pass
