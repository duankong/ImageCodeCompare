#!/usr/bin/env python3
"""
Framework for image compression comparison.

A new codec can be easily added to the framework.
Add the definition to TUPLE_CODECS and implement corresponding encoding, decoding,
and metric calculation steps in method f(). Please use existing codecs as examples.

Our study showing that VMAF has very high correlation with human scores can be found in
IEEE Transactions on Image Processing with article title
"Quality Measurement of Images on Mobile Streaming Interfaces Deployed at Scale".

Our methodology here is to find encoder parameters for a given source image
in order to achieve a given target VMAF quality. Having such encodes
using different codecs that achieve the same VMAF score simplifies comparison
of compression efficiency.
"""
import os
import subprocess
import json
import glob
from collections import Counter
import logging
import uuid
import multiprocessing
import ntpath
import threading
import sqlite3

from utils.utils_common import get_filename_with_temp_folder, make_my_tuple, float_to_int, mkdir_p, listdir_full_path, \
    setup_logging, get_pixel_format_for_metric_computation, get_pixel_format_for_encoding
from utils.sqlcmd import get_create_table_command, get_insert_command
from utils.compression import format_adress, lossy_tuple_codes

from config import args_lossy_compress_high_dynamic_range_config

TOTAL_BYTES = Counter()
TOTAL_METRIC = Counter()
TOTAL_ERRORS = Counter()

LOGGER = logging.getLogger('image.lossycompression_high_dynamic_range')
CONNECTION = None
args_lcc_hdr = args_lossy_compress_high_dynamic_range_config()
WORK_DIR = args_lcc_hdr.work_dir
FROM = format_adress()


def run_program(*args, **kwargs):
    """ run command using subprocess.check_output, collect stdout and stderr together and return
    """

    kwargs.setdefault("stderr", subprocess.STDOUT)
    kwargs.setdefault("shell", True)
    # kwargs.setdefault("universal_newlines", True)
    try:
        output = subprocess.check_output(" ".join(*args), **kwargs)
        return decode(output)

    except subprocess.CalledProcessError as e:
        LOGGER.error("***** ATTENTION : subprocess call crashed: %s\n%s", args, e.output)
        raise


def my_exec(cmd):
    """ helper to choose method for running commands
    """
    return run_program(cmd)


def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def get_dimensions(image):
    """ given a source image, return dimensions and bit-depth
    """
    dimension_cmd = ["identify", '-format', '%w,%h,%z', image]
    res = run_program(dimension_cmd)
    res_split = res.split('\n')
    width, height, depth = res_split[len(res_split) - 1].split(
        ',')  # assuming last line has the goodies; warnings, if any, appear before
    return width, height, depth


def kakadu_encode_helper(image, subsampling, temp_folder, source_yuv, param, codec):
    cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
    my_exec(cmd)

    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mj2')
    # kakadu derives width, height, sub-sampling from file name so avoid confusion with folder name
    cmd = ['{}/kdu_v_compress'.format(FROM.kakadu), '-quiet', '-i',
           ntpath.basename(source_yuv), '-o', ntpath.basename(encoded_file), '-precise', '-rate', '-,3',
           'Creversible=yes', '-tolerance',
           '0']
    if codec == 'kakadu-mse':
        cmd[-2:-2] = ['-no_weights']
    run_program(cmd, cwd=temp_folder)

    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded.yuv')

    for filePath in glob.glob(get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded_*.yuv')):
        try:
            os.remove(filePath)
        except:
            LOGGER.error("Error while deleting file : " + filePath)

    cmd = ['{}/kdu_v_expand'.format(FROM.kakadu), '-quiet', '-i', encoded_file,
           '-o', decoded_yuv]
    my_exec(cmd)

    decoded_yuv_files = glob.glob(get_filename_with_temp_folder(temp_folder, 'special_kakadu_decoded_*.yuv'))
    assert len(decoded_yuv_files) == 1

    decoded_yuv = decoded_yuv_files[0]

    return encoded_file, decoded_yuv, source_yuv


def jpeg_encode_helper(codec, cmd, encoded_file, temp_folder):
    if codec == 'jpeg-mse':
        cmd[1:1] = ['-qt', '1']
    elif codec == 'jpeg-ms-ssim':
        cmd[1:1] = ['-qt', '2']
    elif codec == 'jpeg-im':
        cmd[1:1] = ['-qt', '3']
    elif codec == 'jpeg-hvs-psnr':
        cmd[1:1] = ['-qt', '4']
    run_program(cmd)

    cmd = ['/tools/jpeg/jpeg', encoded_file, get_filename_with_temp_folder(temp_folder, 'decoded.ppm')]
    run_program(cmd)


def compute_vmaf(ref_image, dist_image, width, height, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        use the ffmpeg libvmaf filter to compute vmaf, vif, ssim, and ms_ssim.
    """

    pixel_format = get_pixel_format_for_metric_computation(subsampling)

    log_path = get_filename_with_temp_folder(temp_folder, 'stats_vmaf.json')

    cmd = ['ffmpeg', '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', dist_image,
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', ref_image,
           '-lavfi', 'libvmaf=psnr=true:ssim=true:ms_ssim=true:log_fmt=json:log_path=' + log_path, '-f', 'null', '-']

    run_program(cmd)

    vmaf_log = json.load(open(log_path))

    vmaf_dict = dict()
    vmaf_dict["vmaf"] = vmaf_log["frames"][0]["metrics"]["vmaf"]
    vmaf_dict["vif"] = vmaf_log["frames"][0]["metrics"]["vif_scale0"]
    vmaf_dict["ssim"] = vmaf_log["frames"][0]["metrics"]["ssim"]
    vmaf_dict["ms_ssim"] = vmaf_log["frames"][0]["metrics"]["ms_ssim"]
    vmaf_dict["adm2"] = vmaf_log["frames"][0]["metrics"]["adm2"]
    return vmaf_dict


def compute_psnr(ref_image, dist_image, width, height, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        use the ffmpeg psnr filter to compute psnr and mse for each channel.
    """

    pixel_format = get_pixel_format_for_metric_computation(subsampling)

    log_path = get_filename_with_temp_folder(temp_folder, 'stats_psnr.log')
    cmd = ['ffmpeg', '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', dist_image,
           '-f', 'rawvideo', '-pix_fmt', pixel_format, '-s:v', '%s,%s' % (width, height), '-i', ref_image,
           '-lavfi', 'psnr=stats_file=' + log_path,
           '-f', 'null', '-'
           ]

    run_program(cmd)

    psnr_dict = dict()
    psnr_log = open(log_path).read()
    for stat in psnr_log.rstrip().split(" "):
        key, value = stat.split(":")
        if key != "n":
            psnr_dict[key] = float(value)
    return psnr_dict


def compute_metrics(ref_image, dist_image, width, height, temp_folder, subsampling):
    """ given a pair of reference and distorted images:
        call vmaf and psnr functions, return results in a dict.
    """
    vmaf = compute_vmaf(ref_image, dist_image, width, height, temp_folder, subsampling)
    psnr = compute_psnr(ref_image, dist_image, width, height, temp_folder, subsampling)
    stats = vmaf.copy()
    stats.update(psnr)
    return stats


def f(image, width, height, temp_folder, codec, subsampling, param_lossy):
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
    LOGGER.debug("Encoding image " + image + " with codec " + codec + " LOSSY High Dynamic Range")
    encoded_file = get_filename_with_temp_folder(temp_folder, 'encoded_file_whoami')
    source_yuv = get_filename_with_temp_folder(temp_folder, 'source.yuv')
    decoded_yuv = get_filename_with_temp_folder(temp_folder, 'decoded.yuv')

    if codec in ['jpeg'] and subsampling in ['420', '444']:
        cmd = ['convert', image, get_filename_with_temp_folder(temp_folder, 'source.ppm')]
        run_program(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.jpg')
        cmd = ['/tools/jpeg/jpeg', '-q', param_lossy, '-Q', param_lossy, '-h', '-r', '-s',
               '1x1,2x2,2x2' if subsampling == '420' else '1x1,1x1,1x1',
               get_filename_with_temp_folder(temp_folder, 'source.ppm'),
               encoded_file]
        jpeg_encode_helper(codec, cmd, encoded_file, temp_folder)

        cmd = ['convert', get_filename_with_temp_folder(temp_folder, 'source.ppm'), '-interlace', 'plane',
               '-sampling-factor', '4:2:0' if subsampling == '420' else '4:4:4', source_yuv]
        run_program(cmd)

        cmd = ['convert', get_filename_with_temp_folder(temp_folder, 'decoded.ppm'), '-interlace', 'plane',
               '-sampling-factor', '4:2:0' if subsampling == '420' else '4:4:4', decoded_yuv]
        run_program(cmd)

    elif codec == "webp" and subsampling in ['420']:

        decoded_ppm = get_filename_with_temp_folder(temp_folder, 'decode.ppm')

        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.webp')

        cmd = ['{}/cwebp'.format(FROM.webp), '-z', '9', '-quiet', '-lossless', image, '-o', encoded_file]

        run_program(cmd)

        cmd = ['{}/dwebp'.format(FROM.webp), encoded_file, '-ppm', '-quiet', '-o', decoded_ppm]
        run_program(cmd)

        cmd = ['ffmpeg', '-y', '-i', decoded_ppm, '-pix_fmt', get_pixel_format_for_encoding(subsampling), decoded_yuv]
        run_program(cmd)



    elif codec in ['kakadu-mse', 'kakadu-visual'] and subsampling in ['420']:

        source_yuv = get_filename_with_temp_folder(temp_folder, 'kakadu_{}x{}_{}.yuv'.format(width, height, '420'))

        encoded_file, decoded_yuv, source_yuv = kakadu_encode_helper(image, subsampling, temp_folder, source_yuv,
                                                                     param_lossy,
                                                                     codec)

    elif codec in ['kakadu-mse', 'kakadu-visual'] and subsampling == '444':

        use_ppm_source = False

        if use_ppm_source:
            source_ppm = get_filename_with_temp_folder(temp_folder, 'source.ppm')
            decoded_ppm = get_filename_with_temp_folder(temp_folder, 'decoded.ppm')

            cmd = ['convert', image, source_ppm]
            run_program(cmd)

            encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.mj2')
            cmd = ['/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602/kdu_compress', '-quiet', '-i', source_ppm,
                   '-o', encoded_file, '-rate', '-', '"Ssampling={1,1}"', '-precise', '-tolerance', '0']
            if codec == 'kakadu-mse':
                cmd[-4:-4] = ['-no_weights']
            run_program(cmd)  # not sure whether yuv444 is being coded in the codestream

            cmd = ['/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602/kdu_expand', '-quiet', '-i', encoded_file,
                   '-o', decoded_ppm]
            run_program(cmd)

            cmd = ['convert', source_ppm, '-interlace', 'plane', '-sampling-factor', '4:4:4', source_yuv]
            run_program(cmd)

            cmd = ['convert', decoded_ppm, '-interlace', 'plane', '-sampling-factor', '4:4:4', decoded_yuv]
            run_program(cmd)
        else:
            source_yuv = get_filename_with_temp_folder(temp_folder, 'kakadu_{}x{}_{}.yuv'.format(width, height, '444'))

            encoded_file, decoded_yuv, source_yuv = kakadu_encode_helper(image, subsampling, temp_folder, source_yuv,
                                                                         param_lossy, codec)

    elif codec == 'openjpeg' and subsampling in ['420']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.j2k')
        decoded_file = get_filename_with_temp_folder(temp_folder, 'decoded.ppm')
        source_raw = get_filename_with_temp_folder(temp_folder, 'source.raw')

        # cmd = ['convert', image, '-interlace', 'plane', '-sampling-factor', '4:2:0', source_yuv]
        # run_program(cmd)
        #
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(cmd)
        cmd = ['cp', source_yuv, source_raw]
        run_program(cmd)
        # COMPRESS and DECODE
        cmd = ['opj_compress', '-i', source_raw, '-o', encoded_file, '-F',
               '%s,%s,%s,%s,u@1x1:2x2:2x2' % (width, height, 3, 8), '-r'
            , '{}'.format(param_lossy)]
        run_program(cmd)
        cmd = ['opj_decompress', '-i', encoded_file, '-o', decoded_file]
        run_program(cmd)
        # decode YUV
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format_for_encoding(subsampling), decoded_yuv]
        run_program(cmd)

        # cmd = ['cp', decoded_file, decoded_yuv]
        # run_program(cmd)

    elif codec == 'openjpeg' and subsampling == '444':
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.j2k')
        decoded_file = get_filename_with_temp_folder(temp_folder, 'decoded.raw')
        source_raw = get_filename_with_temp_folder(temp_folder, 'source.raw')

        # cmd = ['convert', image, '-interlace', 'plane', '-sampling-factor', '4:4:4', source_yuv]
        # run_program(cmd)

        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(cmd)

        cmd = ['cp', source_yuv, source_raw]
        run_program(cmd)

        # param is PSNR value [dB]
        cmd = ['opj_compress', '-i', source_raw, '-F', '%s,%s,%s,%s,u@1x1:1x1:1x1' % (width, height, 3, 8), '-r'
            , '{}'.format(param_lossy), '-o', encoded_file]
        run_program(cmd)

        cmd = ['opj_decompress', '-i', encoded_file, '-o', decoded_file]
        run_program(cmd)

        cmd = ['cp', decoded_file, decoded_yuv]
        run_program(cmd)

    elif codec in ['avif-mse', 'avif-ssim'] and subsampling in ['420', '444']:
        param = float_to_int(param_lossy)
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avif')
        cmd = ['aomenc', '--i420' if subsampling == '420' else '--i444', '--width={}'.format(width),
               '--height={}'.format(height), '--ivf', '--cpu-used=1',
               '--end-usage=q', '--lossless=1', '--cq-level={}'.format(param), '--min-q={}'.format(param),
               '--max-q={}'.format(param),
               '--passes=2', '--input-bit-depth={}'.format(8), '--bit-depth={}'.format(8), '--lag-in-frames=0',
               '--frame-boost=0', '--disable-warning-prompt', '--output={}'.format(encoded_file), source_yuv]
        if codec == 'avif-ssim':
            cmd[-2:-2] = ['--tune=ssim']
        run_program(cmd)

        cmd = ['aomdec', '--i420' if subsampling == '420' else '--rawvideo', '-o', decoded_yuv, encoded_file]
        run_program(cmd)

    elif codec in ['bpg'] and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(cmd)
        # bpg encode
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.bpg')
        cmd = ['/tools/libbpg-master/bpgenc', '-lossless', '-f', '420' if subsampling == '420' else '444', '-b', '8',
               '-m', '9', '-o', encoded_file, image]
        run_program(cmd)
        # bpg decoder
        decoded_file = get_filename_with_temp_folder(temp_folder, 'bpg_decode.ppm')
        cmd = ['/tools/libbpg-master/bpgdec', '-b', '8', '-o', decoded_file, encoded_file]
        run_program(cmd)
        # convert
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format_for_encoding(subsampling), decoded_yuv]
        run_program(cmd)

    elif codec in ['flif'] and subsampling in ['420', '444']:
        # flif encode
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.flif')
        if subsampling == '420':
            cmd = ['/tools/FLIF-0.3/src/flif', '-e', '-J', '-m', '-o', '-E', '100', '-Q', param_lossy,
                   image, encoded_file]
        else:
            cmd = ['/tools/FLIF-0.3/src/flif', '-e', '-m', '-o', '-E', '100', '-Q', param_lossy, image, encoded_file]
        run_program(cmd)
        # flif decoder
        decoded_file = get_filename_with_temp_folder(temp_folder, 'flif_decode.png')
        cmd = ['/tools/FLIF-0.3/src/flif', '-d', '-o', '-q', '100', encoded_file, decoded_file]
        run_program(cmd)
        # convert
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(cmd)
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format_for_encoding(subsampling), decoded_yuv]
        run_program(cmd)

    elif codec == 'hevc' and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        my_exec(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['{}/TAppEncoderStatic'.format(FROM.hevc),
               '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_main_rext.cfg', '--CostMode=lossless',
               '-f', '1', '-fr', '1',
               '-wdt', width, '-hgt', height, '--InputChromaFormat={}'.format(subsampling),
               '--ConformanceWindowMode=1', '--TransquantBypassEnable=1', '--CUTransquantBypassFlagForce=1',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(cmd)

        cmd = ['{}/TAppDecoderStatic'.format(FROM.hevc), '-b', encoded_file, '-d', '8', '-o', decoded_yuv]
        my_exec(cmd)

    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling)

    stats = compute_metrics(source_yuv, decoded_yuv, width, height, temp_folder, subsampling)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)

    return stats, encoded_file


def bisection(image, width, height, metric, target, codec, subsampling, param_lossy):
    """ Perform encode with given codec, subsampling, etc. with the goal of hitting given target quality as
    closely as possible. Employs binary search.
    :param inverse: boolean True means QP, else quality factor
    :param ab_tol: how close can a and b get before we exit, used as a terminating condition, just in case.
    :param metric: LOSSY
    :param target: target value of vmaf or PSNR
    :param target_tol: say 2 for VMAF, 0.2 for PSNR
    :param codec: string identifying codec
    :param image: source image
    :param width: width of source image and target encode
    :param height: height of source image and target encode
    :param subsampling: color subsampling
    :return:
    """

    last_c = str(param_lossy)

    temp_uuid = str(uuid.uuid4())
    temp_folder = WORK_DIR + make_my_tuple(LOGGER, image, width, height, codec, metric, target, subsampling,
                                           uuid=temp_uuid)
    tuple_minus_uuid = make_my_tuple(LOGGER, image, width, height, codec, metric, target, subsampling)
    mkdir_p(temp_folder)

    LOGGER.debug(repr((multiprocessing.current_process(), temp_folder,
                       metric, codec, image, width, height, subsampling)))
    quality, encoded_file = f(image, width, height, temp_folder, codec, subsampling, last_c)

    return (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file), subsampling,
            tuple_minus_uuid, ntpath.basename(image), width, height, temp_folder)


def update_stats(results):
    """ callback function called when a worker process finishes an encoding job with target quality value
    """
    c, codec, metric, target, quality, encoded_file, file_size_bytes, subsampling, \
    tuple_minus_uuid, source_image, width, height, temp_folder = results
    LOGGER.info('<<' + codec.upper() + '>>' + " Param " + str(c) + ", quality "
                + repr(quality) + ", encoded_file: " + encoded_file
                + " size: " + str(file_size_bytes) + " bytes")
    TOTAL_BYTES[codec + subsampling + metric + str(target)] += os.path.getsize(encoded_file)
    TOTAL_METRIC[codec + subsampling + metric + str(target)] += quality['psnr_avg']

    try:
        CONNECTION.execute(get_insert_command(), (tuple_minus_uuid, source_image, width, height,
                                                  codec, c, temp_folder, metric, target,
                                                  quality['vmaf'], quality['ssim'], quality['ms_ssim'], quality['vif'],
                                                  quality['mse_y'], quality['mse_u'], quality['mse_v'],
                                                  quality['mse_avg'],
                                                  quality['psnr_y'], quality['psnr_u'], quality['psnr_v'],
                                                  quality['psnr_avg'], quality['adm2'],
                                                  subsampling, file_size_bytes, encoded_file))
        CONNECTION.commit()
    except:
        print("[=============]ERROR")
        # CONNECTION.rollback()
    # remove_files_keeping_encode(temp_folder, encoded_file)  # comment out to keep all files


def error_function(error):
    """ error callback called when an encoding job in a worker process encounters an exception
    """
    LOGGER.error('***** ATTENTION %s', type(error))
    LOGGER.error('***** ATTENTION %s', repr(error))


def initialize_worker():
    """ method called before a worker process picks up jobs
    """
    setup_logging(LOGGER=LOGGER, worker=True, worker_id=multiprocessing.current_process().pid)
    LOGGER.info('initialize_worker() called for %s %s', multiprocessing.current_process(),
                multiprocessing.current_process().pid)


def create_table_if_needed(connection, only_perform_missing_encodes):
    if only_perform_missing_encodes:
        if connection.execute(
                ''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='ENCODES' ''').fetchall()[0][
            0] == 1:
            LOGGER.info('Will add missing entries to table')
        else:
            LOGGER.error('only_perform_missing_encodes is True but table does not exist')
            exit(1)
    else:
        if connection.execute(
                ''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='ENCODES' ''').fetchall()[0][
            0] == 1:
            LOGGER.error('Table already exists. Exiting . . . ')
            exit(1)
        connection.execute(get_create_table_command())


def does_entry_exist(connection, primary_key):
    res = connection.execute("SELECT * FROM ENCODES WHERE ID='{}'".format(primary_key)).fetchall()
    if len(res) > 1:
        LOGGER.error('Found more than one entry with given primary key')
        exit(1)
    elif len(res) == 1:
        return True
    else:
        return False


def main():
    """ create a pool of worker processes and submit encoding jobs, collect results and exit
    """
    setup_logging(LOGGER=LOGGER, worker=False, worker_id=multiprocessing.current_process().ident)
    TUPLE_CODECS = lossy_tuple_codes()

    metric = 'psnr_avg'
    target = 0

    db_file_name = args_lcc_hdr.db_file_name
    only_perform_missing_encodes = args_lcc_hdr.only_perform_missing_encodes

    LOGGER.info(
        'started main, current thread ID %s %s %s', multiprocessing.current_process(),
        multiprocessing.current_process().pid,
        threading.current_thread().ident)

    if only_perform_missing_encodes:
        if os.path.isfile(db_file_name):
            LOGGER.info("Will add missing entries to file " + db_file_name)
        else:
            LOGGER.error("only_perform_missing_encodes is True but db file " + db_file_name + " does not exist.")
            exit(1)

    global CONNECTION
    CONNECTION = sqlite3.connect(db_file_name, check_same_thread=False)
    create_table_if_needed(CONNECTION, only_perform_missing_encodes)

    pool = multiprocessing.Pool(processes=4, initializer=initialize_worker)
    images = set(listdir_full_path('/code/images'))
    if len(images) <= 0:
        LOGGER.error(" no source files in ./images.")
        exit(1)

    results = list()

    for num, image in enumerate(images):
        width, height, depth = get_dimensions(image)
        LOGGER.info(
            "[" + str(num) + "] Source image: " + image + " {" + width + "x" + height + "} bit-depth: " + depth)

        for codec in TUPLE_CODECS:
            LOGGER.debug(" ")
            skip_encode = False
            if only_perform_missing_encodes:
                unique_id = make_my_tuple(LOGGER, image, width, height, codec.name, metric, target,
                                          codec.subsampling)
                skip_encode = does_entry_exist(CONNECTION, unique_id)

            if not skip_encode:
                results.append(
                    (pool.apply_async(bisection,
                                      args=(image, width, height, metric, target,
                                            codec.name, codec.subsampling, codec.param_lossy),
                                      callback=update_stats,
                                      error_callback=error_function),
                     codec.name,
                     codec.subsampling))
        LOGGER.info('-----------------------------------------------------------------------------------------')

    pool.close()
    pool.join()

    # time.sleep(4)

    LOGGER.debug('\n\n')
    LOGGER.debug("Will get results from AsyncResult objects and list them now. This ensures callbacks complete.")
    for result in results:
        try:
            task_result = result[0].get()
            LOGGER.info(repr(task_result))
        except Exception as e:  ## all the exceptions encountered during parallel execution can be collected here at the end nicely
            LOGGER.error(repr(e))
            TOTAL_ERRORS[result[1] + result[2]] += 1

    LOGGER.info("\n\n")
    if not only_perform_missing_encodes:
        LOGGER.info("Total payload in kilo Bytes:")
        for codec in TUPLE_CODECS:
            if codec.name + codec.subsampling in TOTAL_ERRORS:
                LOGGER.info(
                    '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                        TOTAL_BYTES[codec.name + codec.subsampling + metric + str(
                                                            target)] / 1000.0,
                                                        TOTAL_ERRORS[
                                                            codec.name + codec.subsampling + metric + str(target)]))
            else:
                LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                              TOTAL_BYTES[
                                                  codec.name + codec.subsampling + metric + str(target)] / 1000.0))

        LOGGER.info("Average metric value:")
        for codec in TUPLE_CODECS:
            if codec.name + codec.subsampling in TOTAL_ERRORS:
                LOGGER.info(
                    '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                        TOTAL_METRIC[
                                                            codec.name + codec.subsampling + metric + str(target)]
                                                        / float(len(images)),
                                                        TOTAL_ERRORS[
                                                            codec.name + codec.subsampling + metric + str(target)]))
            else:
                LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                              TOTAL_METRIC[
                                                  codec.name + codec.subsampling + metric + str(target)] / float(
                                                  len(images))))

    CONNECTION.close()
    LOGGER.info("\n\n")
    LOGGER.info("[*] --------------------------- Done --------------------------- [*]")
    logging.shutdown()
    os.system("stty echo")
    return


if __name__ == "__main__":
    # if some encodes don't materialize, you can break out with Ctrl+C
    # then comment this out and run below for missing encodes
    main()
    # to run missing encodes
    # main(metric='ssim', target_arr=[0.92, 0.95, 0.97, 0.99], target_tol=0.005, db_file_name='encoding_results_ssim.db', only_perform_missing_encodes=True)
    # main(metric='vmaf', target_arr=[75, 80, 85, 90, 95], target_tol=0.5, db_file_name='encoding_results_vmaf.db', only_perform_missing_encodes=True)

    # Results:
    # Can be easily viewed in Python, say the Python Interactive Console, like so:
    #
    # import sqlite3
    #
    # conn = sqlite3.connect('encoding_results.db')
    # all_results = conn.execute('select * from encodes').fetchall()
    # OR
    # some_results = conn.execute('select codec,sub_sampling,vmaf,file_size_bytes from encodes').fetchall()
