import os
import glob
import ntpath
from .dirs_func import get_filename_with_temp_folder
from .run_cmd import run_program, my_exec
from .UtilsCommon import float_to_int
from .ffmpeg_format import get_pixel_format
from .MetricCaculate import compute_metrics
from .cmd_root_path import cmd_root_path

FORM = cmd_root_path()


def f_image_lossy_8bit(LOGGER, param, codec, image, width, height, temp_folder, subsampling):
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
    param = str(param)
    width = str(width)
    height = str(height)

    encoded_file = get_filename_with_temp_folder(temp_folder, 'encoded_file_whoami')
    source_yuv = get_filename_with_temp_folder(temp_folder, 'source.yuv')
    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'decoded.yuv')
    # 1 JPEG
    if codec in ['jpeg', 'jpeg-mse', 'jpeg-ms-ssim', 'jpeg-im', 'jpeg-hvs-psnr'] and subsampling in ['420', '444']:
        cmd = ['convert', image, get_filename_with_temp_folder(temp_folder, 'source.ppm')]
        run_program(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.jpg')
        cmd = ['{}/jpeg'.format(FORM.jpeg), '-q', float_to_int(param),
               '-s', '1x1,2x2,2x2' if subsampling == '420' else '1x1,1x1,1x1',
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
                                                   'kakadu_{}x{}_{}.yuv'.format(width, height, subsampling))
        encoded_file, decoded_yuv, source_yuv = kakadu_encode_helper(LOGGER, image, subsampling, temp_folder,
                                                                     source_yuv,
                                                                     param, codec)
    # 3 OPENJPEG
    elif codec == 'openjpeg' and subsampling in ['420']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.j2k')
        decoded_file = get_filename_with_temp_folder(temp_folder, 'decoded.ppm')
        source_raw = get_filename_with_temp_folder(temp_folder, 'source.raw')
        # cmd = ['convert', image, '-interlace', 'plane', '-sampling-factor', '4:2:0', source_yuv]
        # run_program(LOGGER,cmd)
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        run_program(LOGGER, cmd)
        cmd = ['cp', source_yuv, source_raw]
        run_program(LOGGER, cmd)
        # param is PSNR value [dB]
        cmd = ['opj_compress', '-i', source_raw, '-F', '%s,%s,%s,%s,u@1x1:2x2:2x2' % (width, height, 3, 8), '-q', param,
               '-o', encoded_file]
        run_program(LOGGER, cmd)
        cmd = ['opj_decompress', '-i', encoded_file, '-o', decoded_file]
        run_program(LOGGER, cmd)
        cmd = ['convert', decoded_file, '-interlace', 'plane', '-sampling-factor', '4:2:0', decoded_yuv]
        run_program(LOGGER, cmd)
    elif codec == 'openjpeg' and subsampling == '444':
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.j2k')
        decoded_file = get_filename_with_temp_folder(temp_folder, 'decoded.raw')
        source_raw = get_filename_with_temp_folder(temp_folder, 'source.raw')
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        run_program(LOGGER, cmd)
        cmd = ['cp', source_yuv, source_raw]
        run_program(LOGGER, cmd)
        # param is PSNR value [dB]
        cmd = ['opj_compress', '-i', source_raw, '-F', '%s,%s,%s,%s,u@1x1:1x1:1x1' % (width, height, 3, 8), '-q', param,
               '-o', encoded_file]
        run_program(LOGGER, cmd)
        cmd = ['opj_decompress', '-i', encoded_file, '-o', decoded_file]
        run_program(LOGGER, cmd)
        cmd = ['cp', decoded_file, decoded_yuv]
        run_program(LOGGER, cmd)
    # 4 FLIF
    elif codec in ['flif'] and subsampling in ['420', '444']:
        # flif encode
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.flif')
        if subsampling == '420':
            cmd = ['/tools/FLIF-0.3/src/flif', '-e', '-J', '-m', '-o', '-E', '100', '-Q', param,
                   image, encoded_file]
        else:
            cmd = ['/tools/FLIF-0.3/src/flif', '-e', '-m', '-o', '-E', '100', '-Q', param, image, encoded_file]
        run_program(LOGGER, cmd)
        # flif decoder
        decoded_file = get_filename_with_temp_folder(temp_folder, 'flif_decode.png')
        cmd = ['/tools/FLIF-0.3/src/flif', '-d', '-o', '-q', '100', encoded_file, decoded_file]
        run_program(LOGGER, cmd)
        # convert
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        run_program(LOGGER, cmd)
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format(subsampling,'8'), decoded_yuv]
        run_program(LOGGER, cmd)
    # 5 WEBP
    elif codec == "webp" and subsampling in ['420']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        run_program(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.webp')
        cmd = ['{}/cwebp'.format(FORM.webp), '-m', '6', '-q', param, '-s', str(width), str(height),
               '-quiet', source_yuv, '-o', encoded_file]
        run_program(LOGGER, cmd)
        cmd = ['{}/dwebp'.format(FORM.webp), encoded_file, '-yuv', '-quiet', '-o', decoded_yuv]
        run_program(LOGGER, cmd)
    # 6 BPG
    elif codec in ['bpg'] and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        run_program(LOGGER, cmd)
        # bpg encode
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.bpg')
        cmd = ['{}/bpgenc'.format(FORM.bpg), '-f', '420' if subsampling == '420' else '444', '-m', '9', '-q',
               str(param), '-o', encoded_file,
               image]
        run_program(LOGGER, cmd)
        # bpg decoder
        decoded_file = get_filename_with_temp_folder(temp_folder, 'bpg_decode.ppm')
        cmd = ['{}/bpgdec'.format(FORM.bpg), '-o', decoded_file, encoded_file]
        run_program(LOGGER, cmd)
        # convert
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format(subsampling,'8'),
               decoded_yuv]
        run_program(LOGGER, cmd)
    # 7 HEVC
    elif codec == 'hevc' and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        my_exec(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['{}/TAppEncoderStatic'.format(FORM.hevc),
               '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_main_rext.cfg', '-f', '1', '-fr', '1', '-q', param,
               '-wdt', width, '-hgt', height, '--InputChromaFormat={}'.format(subsampling),
               '--ConformanceWindowMode=1', '--TransquantBypassEnable=0', '--CUTransquantBypassFlagForce=0',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(LOGGER, cmd)
        cmd = ['{}/TAppDecoderStatic'.format(FORM.hevc), '-b', encoded_file, '-d', '8', '-o', decoded_yuv]
        my_exec(LOGGER, cmd)
    # 8 AVIF
    elif codec in ['avif-mse', 'avif-ssim'] and subsampling in ['420', '444']:
        param = float_to_int(param)
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        run_program(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avif')
        cmd = ['aomenc', '--i420' if subsampling == '420' else '--i444', '--width={}'.format(width),
               '--height={}'.format(height), '--ivf', '--cpu-used=1',
               '--end-usage=q', '--cq-level={}'.format(param), '--min-q={}'.format(param), '--max-q={}'.format(param),
               '--passes=2', '--input-bit-depth={}'.format(8), '--bit-depth={}'.format(8), '--lag-in-frames=0',
               '--frame-boost=0', '--disable-warning-prompt',
               '--output={}'.format(encoded_file), source_yuv]
        if codec == 'avif-ssim':
            cmd[-2:-2] = ['--tune=ssim']
        run_program(LOGGER, cmd)
        cmd = ['aomdec', '--i420' if subsampling == '420' else '--rawvideo', '-o', decoded_yuv, encoded_file]
        run_program(LOGGER, cmd)
    # 9 LIBAVIF
    elif codec in ['avifenc-sp-0', 'avifenc-sp-2', 'avifenc-sp-4', 'avifenc-sp-8'] and subsampling in ['420', '444']:
        param = float_to_int(param)
        min_QP = int(param) - 1
        max_QP = int(param) + 1
        info = codec.split('-')
        speed = info[2]
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
        my_exec(LOGGER, cmd)
        source_y4m = get_filename_with_temp_folder(temp_folder, 'source.y4m')
        cmd = ['ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', get_pixel_format(subsampling,'8'),
               '-s:v', '%s,%s' % (width, height), '-i', source_yuv, source_y4m]
        my_exec(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avif')
        # bit-depth and yuv subsampling are maintained for y4m input
        cmd = ['/tools/libavif/build/avifenc', source_y4m, encoded_file, '--depth', '8', '--nclx', '1/13/1', '--min',
               str(min_QP),
               '--max', str(max_QP), '--speed', speed, '--codec', 'aom', '--jobs', '4']
        my_exec(LOGGER, cmd)
        decoded_y4m = get_filename_with_temp_folder(temp_folder, 'decoded.y4m')
        cmd = ['/tools/libavif/build/avifdec', '--codec', 'aom', encoded_file, decoded_y4m]
        my_exec(LOGGER, cmd)
        # explicitly convert to 420 or 444 depending on the case from the y4m
        cmd = ['ffmpeg', '-y', '-i', decoded_y4m, '-pix_fmt', get_pixel_format(subsampling,'8'),
               decoded_yuv]
        my_exec(LOGGER, cmd)
    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling)
    stats = compute_metrics(LOGGER, source_yuv, decoded_yuv, width, height, '8', temp_folder, subsampling)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)
    LOGGER.debug(
        "Encoding  {} with {:<13} param={:<13} subsampling={}|| "
        "psnr_avg={:<8} ssim={:<8}  vmaf={:<8} ||"
        " size={} ".format(image, codec,
                           param[0:12],
                           subsampling,
                           stats['psnr_avg'],
                           stats['ssim'],
                           stats['vmaf'],
                           stats['file_size_bytes']))
    return stats, encoded_file


def kakadu_encode_helper(LOGGER, image, subsampling, temp_folder, source_yuv, param, codec):
    cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format(subsampling,'8'), source_yuv]
    my_exec(LOGGER, cmd)

    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mj2')
    # kakadu derives width, height, sub-sampling from file name so avoid confusion with folder name
    cmd = ['{}/kdu_v_compress'.format(FORM.kakadu), '-quiet', '-i',
           ntpath.basename(source_yuv), '-o', ntpath.basename(encoded_file), '-precise', '-rate', param, '-tolerance',
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

    cmd = ['{}/kdu_v_expand'.format(FORM.kakadu), '-quiet', '-i', encoded_file, '-o', decoded_yuv]
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
