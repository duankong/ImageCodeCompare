import os
import glob
import ntpath
from .dirs_func import get_filename_with_temp_folder
from .run_cmd import run_program, my_exec
from .ffmpeg_format import get_pixel_format
from .MetricCaculate import compute_metrics
from .cmd_root_path import cmd_root_path

FROM = cmd_root_path()


def f_image_lossless_16bit(LOGGER, image, width, height, temp_folder, codec, subsampling, param_lossy):
    """
    Method to run an encode with given codec, subsampling, etc. and requested codec parameter like QP or bpp.
    All the commands for encoding, decoding, computing metric, etc. for any given codec should be visible
    in one contiguous block of code. Don't care about repeated commands across codecs.
    :param param: codec param like QP, bpp, etc.
    :param codec: codec
    :param image: filename for source image
    :param width: width of source image and target encode
    :param height: height of source image and target encode
    :param temp_folder: directory for intermediate files, encodes, stats files, etc.
    :param subsampling: color subsampling
    :return:
    """
    LOGGER.debug("Encoding image " + image + " with codec " + codec + " LOSSLESS ")
    encoded_file = get_filename_with_temp_folder(temp_folder, 'encoded_file_whoami')
    source_yuv = get_filename_with_temp_folder(temp_folder, 'source.yuv')
    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'decoded.yuv')
    # 1 JPEG
    if codec in ['jpeg'] and subsampling in ['420', '444']:
        cmd = ['convert', image, get_filename_with_temp_folder(temp_folder, 'source.ppm')]
        run_program(LOGGER, cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.jpg')
        cmd = ['/tools/jpeg/jpeg', '-xyz', '-q', '100', '-Q', '100', '-h', '-r', '-s',
               '1x1,2x2,2x2' if subsampling == '420' else '1x1,1x1,1x1',
               get_filename_with_temp_folder(temp_folder, 'source.ppm'), encoded_file]
        jpeg_encode_helper(LOGGER, codec, cmd, encoded_file, temp_folder)

        cmd = ['convert', get_filename_with_temp_folder(temp_folder, 'source.ppm'), '-interlace', 'plane',
               '-sampling-factor', '4:2:0' if subsampling == '420' else '4:4:4', source_yuv]
        run_program(LOGGER, cmd)

        cmd = ['convert', get_filename_with_temp_folder(temp_folder, 'decoded.ppm'), '-interlace', 'plane',
               '-sampling-factor', '4:2:0' if subsampling == '420' else '4:4:4', decoded_yuv]
        run_program(LOGGER, cmd)
    # 2 KAKADU
    elif codec in ['kakadu-mse', 'kakadu-visual'] and subsampling in ['420', '444']:

        source_yuv = get_filename_with_temp_folder(temp_folder,
                                                   'kakadu_{}x{}_{}_{}_{}.yuv'.format(width, height, '60', '16b',
                                                                                      subsampling))

        encoded_file, decoded_yuv, source_yuv = kakadu_encode_helper_16bit(LOGGER, image, subsampling, temp_folder,
                                                                           source_yuv, None, codec)
    # 3 OPENJPEG
    elif codec == 'openjpeg' and subsampling in ['420', '444']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.j2k')
        decoded_file = get_filename_with_temp_folder(temp_folder, 'decoded.ppm')
        source_raw = get_filename_with_temp_folder(temp_folder, 'source.raw')

        cmd = ['convert', image, '-interlace', 'plane', '-sampling-factor',
               '4:2:0' if subsampling == '420' else '4:4:4', source_yuv]
        run_program(LOGGER, cmd)

        cmd = ['cp', source_yuv, source_raw]
        run_program(LOGGER, cmd)

        # COMPRESS and DECODE
        cmd = ['opj_compress', '-i', source_raw, '-o', encoded_file, '-F',
               '{},{},{},{},u@{}'.format(width, height, 1, 16,
                                         '1x1:2x2:2x2' if subsampling == '420' else '1x1:1x1:1x1'),
               '-r', '{}'.format(1)]
        run_program(LOGGER, cmd)
        cmd = ['opj_decompress', '-i', encoded_file, '-o', decoded_file]
        run_program(LOGGER, cmd)
        # decode YUV

        cmd = ['convert', decoded_file, '-interlace', 'plane', '-sampling-factor',
               '4:2:0' if subsampling == '420' else '4:4:4', decoded_yuv]
        run_program(LOGGER, cmd)

    # 4 FLIF
    elif codec in ['flif'] and subsampling in ['420', '444']:
        # flif encode
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.flif')
        if subsampling == '420':
            cmd = ['/tools/FLIF-0.3/src/flif', '-e', '-J', '-m', '-o', '-E', '100', '-Q', '100',
                   image, encoded_file]
        else:
            cmd = ['/tools/FLIF-0.3/src/flif', '-e', '-m', '-o', '-E', '100', '-Q', '100', image, encoded_file]
        run_program(LOGGER, cmd)
        # flif decoder
        decoded_file = get_filename_with_temp_folder(temp_folder, 'flif_decode.png')
        cmd = ['/tools/FLIF-0.3/src/flif', '-d', '-o', '-q', '100', encoded_file, decoded_file]
        run_program(LOGGER, cmd)
        # convert
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'16'), source_yuv]
        run_program(LOGGER, cmd)
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format(subsampling,'16'),
               decoded_yuv]
        run_program(LOGGER, cmd)
    # 7 HEVC
    elif codec == 'hevc' and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'16'), source_yuv]
        my_exec(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['{}/TAppEncoderHighBitDepthStatic'.format(FROM.highbithevc),
               '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_high_throughput_rext.cfg',
               '-wdt', width, '-hgt', height,
               '--InputChromaFormat={}'.format(subsampling),
               '--SNRInternalColourSpace=1',
               '--FrameRate=1',  # Frame Rate per second
               '--FramesToBeEncoded=1',  # Number of frames to be coded
               '--InputBitDepth=16',
               '--InternalBitDepth=16',
               '--CostMode=lossless',
               '--ConformanceWindowMode=1',
               '--TransquantBypassEnable=1',
               '--CUTransquantBypassFlagForce=1',
               '--Level=8.5',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(LOGGER, cmd)

        cmd = ['{}/TAppDecoderHighBitDepthStatic'.format(FROM.highbithevc),
               '--OutputBitDepthC=16',
               '--OutputBitDepth=16',
               '-b', encoded_file,
               '-o', decoded_yuv]
        my_exec(LOGGER, cmd)
    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling)
    stats = compute_metrics(LOGGER, source_yuv, decoded_yuv, width, height, '16', temp_folder, subsampling)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)
    return stats, encoded_file


def kakadu_encode_helper_16bit(LOGGER, image, subsampling, temp_folder, source_yuv, param, codec):
    cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'16'), source_yuv]
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
