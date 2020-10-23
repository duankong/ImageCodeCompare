import os
from .dirs_func import get_filename_with_temp_folder
from .run_cmd import run_program, my_exec
from .MetricCaculate import compute_metrics
from .cmd_root_path import cmd_root_path, config_file_root_path
from .ffmpeg_format import get_pixel_format

FORM = cmd_root_path()
config_path = config_file_root_path()


def f_video_lossly_8bit(LOGGER, codec, yuv_status, param, temp_folder):
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
    if codec in ['avc'] and subsampling in ['420', '444']:
        param = str(int(float(param)))
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mp4')
        cmd = ['ffmpeg', '-s', '{}x{}'.format(width, height), '-pix_fmt', get_pixel_format(subsampling, depth),
               '-i', source_yuv, '-c:v', 'libx264',
               '-crf', param, encoded_file, '-y']
        run_program(LOGGER, cmd)
        cmd = ['ffmpeg', ' -i', encoded_file,'-y', decoded_yuv]
        run_program(LOGGER, cmd)

    # 1 HEVC
    elif codec in ['hevc'] and subsampling in ['420', '444']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['{}/TAppEncoderStatic'.format(FORM.hevc),
               # '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_high_throughput_rext.cfg',
               '-c', '{}/encoder_randomaccess_main_rext.cfg'.format(config_path.hevc),
               '-wdt', width, '-hgt', height,
               '--InputChromaFormat={}'.format(subsampling),
               '--SNRInternalColourSpace=1',
               '--FrameRate=1',  # Frame Rate per second
               '--FramesToBeEncoded={}'.format(frames),  # Number of frames to be coded
               '-q', param,
               '--InputBitDepth={}'.format(depth),
               '--InternalBitDepth={}'.format(depth),
               '--CostMode=lossy',
               '--ConformanceWindowMode=1',
               '--TransquantBypassEnable=0',
               '--CUTransquantBypassFlagForce=0',
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
    elif codec in ['vvc'] and subsampling in ['420', '444']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.vvc')
        cmd = ['{}/EncoderAppStatic'.format(FORM.vvc),
               '-c', '{}/encoder_randomaccess_vtm.cfg'.format(config_path.hevc),
               '-wdt', width, '-hgt', height,
               '--InputChromaFormat={}'.format(subsampling),
               '--SNRInternalColourSpace=1',
               '--FrameRate=30',  # Frame Rate per second
               '--FramesToBeEncoded={}'.format(frames),  # Number of frames to be coded
               '-q', param,
               '--InputBitDepth={}'.format(depth),
               '--InternalBitDepth={}'.format(depth),
               '--CostMode=lossy',
               '--ConformanceWindowMode=1',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(LOGGER, cmd)
        cmd = ['{}/DecoderAppStatic'.format(FORM.vvc),
               '--OutputBitDepthC={}'.format(depth),
               '--OutputBitDepth={}'.format(depth),
               '-b', encoded_file,
               '-o', decoded_yuv]
        my_exec(LOGGER, cmd)
    # 3 AVS3
    elif codec in ['avs3'] and subsampling in ['420', '444']:
        param = str(int(float(param)))
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avs3')
        cmd = ['{}/uavs3enc'.format(FORM.avs3enc),
               '--config', '{}/config.cfg'.format('/code/build'),
               '-i', source_yuv, '-w', str(width), '-h', str(height), '-d', str(depth), '-f', str(frames),
               '-q', param,
               '--fps_num', '30',
               '--fps_den', '1',
               '--wpp_threads', '8',
               '--frm_threads', '12',
               '-o', encoded_file]
        run_program(LOGGER, cmd)

        cmd = ['{}/uavs3dec'.format(FORM.avs3dec),
               '--input', encoded_file,
               '--threads', '8',
               '--frames', str(frames),
               '--output', decoded_yuv]
        run_program(LOGGER, cmd)
    # 4 AOM
    elif codec in ['aom-mse', 'aom-ssim'] and subsampling in ['420', '444']:
        param = str(int(float(param)))
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.ivf')
        cmd = ['aomenc', '--i420' if subsampling == '420' else '--i444',
               '--width={}'.format(width), '--height={}'.format(height),
               '--input-bit-depth={}'.format(depth), '--bit-depth={}'.format(depth),
               '--cq-level={}'.format(param), '--min-q={}'.format(param), '--max-q={}'.format(param),
               '--ivf',
               '--good',
               '--passes=2',
               '--cpu-used=8',
               '--end-usage=q',
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
    # 5 rav1e
    elif codec in ['rav1e-Psychovisual', 'rav1e-Psnr'] and subsampling in ['420', '444']:
        param = str(int(float(param)))
        decoded_yuv = get_filename_with_temp_folder(temp_folder, '{}x{}x{}.yuv'.format(width, height, frames))
        y4msouce = get_filename_with_temp_folder(temp_folder, 'souce.y4m')
        cmd = ['ffmpeg', '-s', '{}x{}'.format(width, height), '-pix_fmt', get_pixel_format(subsampling, depth), '-i',
               source_yuv, '-vsync', '0', y4msouce, '-y']
        decoded_y4m = get_filename_with_temp_folder(temp_folder, 'decode.y4m')
        run_program(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.ivf')
        cmd = ['{}/rav1e'.format(FORM.rav1e),
               '--threads=8',
               '--limit={}'.format(frames),
               '--quantizer={}'.format(param),  # Quantizer (0-255), smaller values are higher quality [default: 100]
               '--min-quantizer=0',
               '--speed=7',  # (0 is best quality, 10 is fastest)
               '-r', decoded_y4m,
               '--frame-rate=30', y4msouce,
               '--output', encoded_file, ]
        if codec == 'rav1e-Psnr':
            cmd[-3:-3] = ['--tune=Psnr']
        run_program(LOGGER, cmd)
        cmd = ['aomdec', '--i420' if subsampling == '420' else '--rawvideo', '-o', decoded_yuv, encoded_file]
        run_program(LOGGER, cmd)
    # 6 SVT-AV1
    elif codec in ['svt-av1'] and subsampling in ['420']:
        param = str(param)
        y4msouce = get_filename_with_temp_folder(temp_folder, 'souce.y4m')
        cmd = ['ffmpeg', '-s', '{}x{}'.format(width, height), '-pix_fmt', get_pixel_format(subsampling, depth), '-i',
               source_yuv, '-vsync', '0', y4msouce, '-y']
        run_program(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'svt_av1.ivf')
        cmd = ['{}/SvtAv1EncApp'.format(FORM.svt_av1),
               # '-c', '/code/ImageFormat_SourceCode/VIDEO/AOM/SVTAV1/SVT-AV1-0.8.5/Config/Sample.cfg',
               '--width', width, '--height', height, '--input-depth', depth, '--frames', str(frames),
               '--fps', '30',
               '--rc', '0',  # Rate control mode(0 = CQP , 1 = VBR , 2 = CVBR)
               '--qp', '{}'.format(param),  # quantizer[0-63]
               '--preset', '8',  # Encoder mode/Preset used[-2,-1,0,..,8]
               '--lp', '4',
               '--color-format', '1',  # 420
               '--pred-struct', '2',  # 0: low delay P, 1: low delay B, 2: random access [default]
               '--hierarchical-levels', '5',
               '--input', source_yuv,
               '--output', encoded_file, ]
        if codec == 'rav1e-Psnr':
            cmd[-3:-3] = ['--tune=Psnr']
        run_program(LOGGER, cmd, cwd=temp_folder)
        cmd = ['{}/SvtAv1DecApp'.format(FORM.svt_av1),
               '-i', encoded_file,
               '-o', decoded_yuv,
               '-w', width,
               '-h', height,
               '-bit-depth', '8',
               '-colour-space', subsampling,
               '-limit', str(frames),
               ]
        run_program(LOGGER, cmd)
    # 7 VP8
    elif codec in ['vp8-ssim', 'vp8-psnr', 'vp9-ssim', 'vp9-psnr'] and subsampling in ['420', '444']:
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
               '--max-q={}'.format(int(param) + 4),
               '--min-q={}'.format(int(param) - 4),
               '--end-usage=cq',  # vbr, cbr, cq, q
               '--cq-level={}'.format(param),  # quantizer[4-63]
               '-o', encoded_file, source_yuv]
        run_program(LOGGER, cmd, cwd=temp_folder)

        cmd = ['{}/vpxdec'.format(FORM.vpx),
               '--i420' if subsampling == '420' else '--rawvideo',
               '--limit={}'.format(frames),
               '--threads=4',
               '-o', decoded_yuv, encoded_file]
        run_program(LOGGER, cmd, cwd=temp_folder)
    # 9 SVT-VP9
    elif codec in ['svt-vp9-visually', 'svt-vp9-PSNR-SSIM', 'svt-vp9-VMAF'] and subsampling in ['420']:
        param = str(param)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'svt_vp9.ivf')
        cmd = ['{}/SvtVp9EncApp'.format(FORM.svt_vp9),
               '-w', width, '-h', height, '-bit-depth', depth, '-n', str(frames),
               '-fps', '30',
               '-rc', '0',  # 0 = CQP , 1 = VBR , 2 = CBR
               '-q', param,  # quantizer[0-63]
               '-pred-struct', '2',  # [2] 2: random access [default]
               '-enc-mode', '0',  # [0- 9] 0 is the highest quality mode, 9 is the highest density mode
               '-i', source_yuv,
               '-b', encoded_file,
               '-o', decoded_yuv]
        # 0 = SQ - visually optimized ,1 = OQ - PSNR / SSIM optimized ,2 = VMAF - VMAF optimized
        if codec == 'svt-vp9-visually':
            cmd[-6:-6] = ['-tune 0']
        elif codec == 'svt-vp9-PSNR-SSIM':
            cmd[-6:-6] = ['-tune 1']
        elif codec == 'svt-vp9-VMAF':
            cmd[-6:-6] = ['-tune 2']
        else:
            LOGGER.error('Wrong with codec = {}'.format(codec))
        run_program(LOGGER, cmd, cwd=temp_folder)

    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling)

    stats = compute_metrics(LOGGER, source_yuv, decoded_yuv, width, height, depth, temp_folder, subsampling)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)
    LOGGER.debug(
        "Encoding  {} with {:<20} param={:<8}  frames={:<3} subsampling={}|| "
        "psnr_avg={:<8} ssim={:<8}  vmaf={:<8} || "
        "size={} ".format(yuvsource, codec,
                          str(param)[0:8],
                          frames,
                          subsampling,
                          str(stats['psnr_avg'])[0:8],
                          stats['ssim'],
                          stats['vmaf'],
                          stats['file_size_bytes']))
    return stats, encoded_file


if __name__ == '__main__':
    pass
