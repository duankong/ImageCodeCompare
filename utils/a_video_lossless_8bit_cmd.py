import os
from .u_folder_build import get_filename_with_temp_folder
from .u_run_cmd import run_program, my_exec
from .m_metric_caculate import compute_metrics
from .u_format_bin_root_path import cmd_root_path, config_file_root_path
from .u_ffmpeg_format import get_pixel_format

FORM = cmd_root_path()
config_file = config_file_root_path()


def f_video_lossless_8bit(LOGGER, codec, yuv_status, param, temp_folder):
    """ [('yuv_file', yuv_file), ('width', width), ('height', height), ('depth', depth), ('frames', real_frames),
         ('subsampling', subsampling)])
    Method to run an encode with given codec, subsampling, etc. and requested codec parameter like QP or bpp.
    All the commands for encoding, decoding, computing metric, etc. for any given codec should be visible
    in one contiguous block of code. Don't care about repeated commands across codecs.
    :param param: codec param like QP, bpp, etc.
    :param codec: codec
    :param yuvsource: filename for source image
    :param width: width of source image and target encode
    :param height: height of source image and target encode
    :param temp_folder: directory for intermediate files, encodes, stats files, etc.
    :param subsampling: color subsampling
    :return:
    """
    param = str(param)

    yuvsource = yuv_status['yuv_file']
    subsampling = yuv_status['subsampling']
    width = yuv_status['width']
    height = yuv_status['height']
    frames = yuv_status['frames']
    depth = yuv_status['depth']

    source_yuv = yuvsource + '_{}.yuv'.format(subsampling)
    cmd = ['cp', source_yuv, temp_folder]
    my_exec(LOGGER, cmd)

    encoded_file = get_filename_with_temp_folder(temp_folder, 'encoded_file_whoami')
    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'decoded.yuv')

    # 0 AVC
    if codec in ['avc'] and subsampling in ['420', '444']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mp4')
        cmd = ['ffmpeg', '-s', '{}x{}'.format(width, height), '-pix_fmt', get_pixel_format(subsampling, depth), '-i',
               source_yuv, '-c:v', 'libx264',
               # '-preset', 'veryslow',
               '-crf', '0', encoded_file]
        run_program(LOGGER, cmd)
        cmd = ['ffmpeg', ' -i', encoded_file, decoded_yuv]
        run_program(LOGGER, cmd)

    # ## 1 HEVC
    elif codec == 'hevc' and subsampling in ['420', '444']:

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['{}/TAppEncoderStatic'.format(FORM.hevc),
               '-c', '{}/encoder_randomaccess_main_rext.cfg'.format(config_file.hevc),
               '--CostMode=lossless',
               '-wdt', width, '-hgt', height,
               '--InputChromaFormat={}'.format(subsampling),
               '--SNRInternalColourSpace=1',
               '--FrameRate=1',  # Frame Rate per second
               '--FramesToBeEncoded={}'.format(frames),  # Number of frames to be coded
               '--InputBitDepth={}'.format(depth),
               '--InternalBitDepth={}'.format(depth),
               '--ConformanceWindowMode=1',
               '--TransquantBypassEnable=1',
               '--CUTransquantBypassFlagForce=1',
               '--Level=6.2',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(LOGGER, cmd)
        cmd = ['{}/TAppDecoderStatic'.format(FORM.hevc),
               '--OutputBitDepthC={}'.format(depth),
               '--OutputBitDepth={}'.format(depth),
               '-b', encoded_file,
               '-o', decoded_yuv]
        my_exec(LOGGER, cmd)
    # 2 VVC
    elif codec == 'vvc' and subsampling in ['420', '444']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.vvc')
        cmd = ['{}/EncoderAppStatic'.format(FORM.vvc),
               '-c', '{}/encoder_randomaccess_vtm.cfg'.format(config_file.vvc),
               '-c', '{}/lossless/{}'.format(config_file.vvc,
                                             'lossless.cfg' if subsampling == '420' else 'lossless444.cfg'),
               '--CostMode=lossless',
               '-wdt', width, '-hgt', height,
               '--InputChromaFormat={}'.format(subsampling),
               '--SNRInternalColourSpace=0',
               '--FrameSkip=0',
               '--FrameRate=30',  # Frame Rate per second
               '--FramesToBeEncoded={}'.format(frames),  # Number of frames to be coded
               '--InputBitDepth={}'.format(depth),
               '--InternalBitDepth={}'.format(depth),
               '--ConformanceWindowMode=1',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(LOGGER, cmd)
        cmd = ['{}/DecoderAppStatic'.format(FORM.vvc),
               '--OutputBitDepthC={}'.format(depth),
               '--OutputBitDepth={}'.format(depth),
               '-b', encoded_file,
               '-o', decoded_yuv]
        my_exec(LOGGER, cmd)
    # 4 AOM
    elif codec in ['aom-mse', 'aom-ssim'] and subsampling in ['420', '444']:
        param = str(int(float(param)))
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.ivf')
        cmd = ['aomenc', '--i420' if subsampling == '420' else '--i444',
               '--width={}'.format(width), '--height={}'.format(height),
               '--input-bit-depth={}'.format(depth), '--bit-depth={}'.format(depth),
               '--lossless=1',
               # '--cq-level={}'.format(param), '--min-q={}'.format(param), '--max-q={}'.format(param),
               '--ivf',
               '--good',
               '--passes=2',
               '--cpu-used=8',
               '--lag-in-frames=0',
               '--frame-boost=0',
               '--threads=8',
               '--disable-warning-prompt',
               '--output={}'.format(encoded_file), source_yuv]
        if codec == 'avif-ssim':
            cmd[-2:-2] = ['--tune=ssim']
        run_program(LOGGER, cmd)
        cmd = ['aomdec', '--i420' if subsampling == '420' else '--rawvideo', '-o', decoded_yuv, encoded_file]
        run_program(LOGGER, cmd)
    # 8 VP9
    elif codec in ['vp9-ssim', 'vp9-psnr'] and subsampling in ['420', '444']:
        info = codec.split('-')
        param = str(int(float(param)))
        encoded_file = get_filename_with_temp_folder(temp_folder, '{}.ivf'.format(codec))
        decoded_yuv = get_filename_with_temp_folder(temp_folder, '{}x{}x{}.yuv'.format(width, height, frames))
        cmd = ['{}/vpxenc'.format(FORM.vpx),
               '--codec={}'.format(info[0]),
               '-w', width, '-h', height, '--bit-depth={}'.format(depth), '--limit={}'.format(str(frames)),
               '--threads=4',
               '--ivf',
               '--fps=30/1',
               '--tune={}'.format(info[1]),
               '--i{}'.format(subsampling),
               '--profile={}'.format('0' if subsampling == '420' else '1'),
               '--lossless=1',
               '-o', encoded_file, source_yuv]
        run_program(LOGGER, cmd, cwd=temp_folder)

        cmd = ['{}/vpxdec'.format(FORM.vpx),
               '--i420' if subsampling == '420' else '--rawvideo',
               '--limit={}'.format(frames),
               '--threads=4',
               '-o', decoded_yuv, encoded_file]
        run_program(LOGGER, cmd, cwd=temp_folder)

    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling + 'lossless')

    stats = compute_metrics(LOGGER, source_yuv, decoded_yuv, width, height, depth, temp_folder, subsampling)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)
    LOGGER.debug(
        "Encoding  {} with {:<13} param={:<8} frames={} subsampling={}|| "
        "psnr_avg={:<8} ssim={:<8}  vmaf={:<8} || "
        "size={} ".format(yuvsource, codec,
                          param[0:8],
                          frames,
                          subsampling,
                          str(stats['psnr_avg'])[0:8],
                          stats['ssim'],
                          stats['vmaf'],
                          stats['file_size_bytes']))
    return stats, encoded_file
