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
import glob
from collections import Counter
import logging
import uuid
import multiprocessing
import ntpath
import threading
import sqlite3

from utils.UtilsCommon import get_filename_with_temp_folder, make_my_tuple, float_to_int, mkdir_p, listdir_full_path, \
    setup_logging, get_pixel_format_for_encoding
from utils.MysqlFun import get_insert_command, create_table_if_needed, does_entry_exist
from utils.cmd_root_path import cmd_root_path, tuple_codes
from utils.MetricCaculate import compute_metrics
from utils.run_cmd import run_program, my_exec
from utils.FormatHelp import get_dimensions

from Config.config_compress import args_lossy_compress_config, show_and_recode_compress_args

TOTAL_BYTES = Counter()
TOTAL_METRIC = Counter()
TOTAL_ERRORS = Counter()

LOGGER = logging.getLogger('image.compression')
CONNECTION = None
args = args_lossy_compress_config()
WORK_DIR = args.work_dir
FORM = cmd_root_path()


def kakadu_encode_helper(image, subsampling, temp_folder, source_yuv, param, codec):
    cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
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


def jpeg_encode_helper(codec, cmd, encoded_file, temp_folder):
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


def f(param, codec, image, width, height, temp_folder, subsampling):
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
        jpeg_encode_helper(codec, cmd, encoded_file, temp_folder)
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
        encoded_file, decoded_yuv, source_yuv = kakadu_encode_helper(image, subsampling, temp_folder, source_yuv,
                                                                     param, codec)
    # 3 OPENJPEG
    elif codec == 'openjpeg' and subsampling in ['420']:
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.j2k')
        decoded_file = get_filename_with_temp_folder(temp_folder, 'decoded.ppm')
        source_raw = get_filename_with_temp_folder(temp_folder, 'source.raw')
        # cmd = ['convert', image, '-interlace', 'plane', '-sampling-factor', '4:2:0', source_yuv]
        # run_program(LOGGER,cmd)
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
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
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
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
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(LOGGER, cmd)
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format_for_encoding(subsampling), decoded_yuv]
        run_program(LOGGER, cmd)
    # 5 WEBP
    elif codec == "webp" and subsampling in ['420']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        run_program(LOGGER, cmd)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.webp')
        cmd = ['{}/cwebp'.format(FORM.webp), '-m', '6', '-q', param, '-s', str(width), str(height),
               '-quiet', source_yuv, '-o', encoded_file]
        run_program(LOGGER, cmd)
        cmd = ['{}/dwebp'.format(FORM.webp), encoded_file, '-yuv', '-quiet', '-o', decoded_yuv]
        run_program(LOGGER, cmd)
    # 6 BPG
    elif codec in ['bpg'] and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
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
        cmd = ['ffmpeg', '-y', '-i', decoded_file, '-pix_fmt', get_pixel_format_for_encoding(subsampling),
               decoded_yuv]
        run_program(LOGGER, cmd)
    # 7 HEVC
    elif codec == 'hevc' and subsampling in ['420', '444']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
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
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
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
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        my_exec(LOGGER, cmd)
        source_y4m = get_filename_with_temp_folder(temp_folder, 'source.y4m')
        cmd = ['ffmpeg', '-y', '-f', 'rawvideo', '-pix_fmt', get_pixel_format_for_encoding(subsampling),
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
        cmd = ['ffmpeg', '-y', '-i', decoded_y4m, '-pix_fmt', get_pixel_format_for_encoding(subsampling),
               decoded_yuv]
        my_exec(LOGGER, cmd)
    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling)

    stats = compute_metrics(LOGGER, source_yuv, decoded_yuv, width, height, 8, temp_folder, subsampling)
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

def bisection(inverse, a, b, ab_tol, metric, target, target_tol, codec, image, width, height, subsampling):
    """ Perform encode with given codec, subsampling, etc. with the goal of hitting given target quality as
    closely as possible. Employs binary search.
    :param inverse: boolean True means QP, else quality factor
    :param a: a should be less than b
    :param b: far-end of codec parameter range
    :param ab_tol: how close can a and b get before we exit, used as a terminating condition, just in case.
    :param metric: string, vmaf or PSNR
    :param target: target value of vmaf or PSNR
    :param target_tol: say 2 for VMAF, 0.2 for PSNR
    :param codec: string identifying codec
    :param image: source image
    :param width: width of source image and target encode
    :param height: height of source image and target encode
    :param subsampling: color subsampling
    :return:
    """
    temp_uuid = str(uuid.uuid4())
    temp_folder = WORK_DIR + make_my_tuple(LOGGER, image, width, height, codec, metric, target, subsampling,
                                           uuid=temp_uuid)
    tuple_minus_uuid = make_my_tuple(LOGGER, image, width, height, codec, metric, target, subsampling)
    mkdir_p(temp_folder)

    LOGGER.debug(repr((multiprocessing.current_process(), temp_folder,
                       inverse, a, b, ab_tol, metric, target, target_tol, codec, image, width, height, subsampling)))

    c = (a + b) / 2
    last_c = c
    while (b - a) > ab_tol:

        quality, encoded_file = f(c, codec, image, width, height, temp_folder, subsampling)
        last_c = c

        if abs(quality[metric] - target) < target_tol:
            return (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file), subsampling,
                    tuple_minus_uuid, ntpath.basename(image), width, height, temp_folder)
        else:
            if inverse:
                if quality[metric] < target:
                    b = c
                else:
                    a = c
            else:
                if quality[metric] < target:
                    a = c
                else:
                    b = c
            c = (a + b) / 2

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
    TOTAL_METRIC[codec + subsampling + metric + str(target)] += quality[metric]
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


def main():
    """ create a pool of worker processes and submit encoding jobs, collect results and exit
    """
    setup_logging(LOGGER=LOGGER, worker=False, worker_id=multiprocessing.current_process().ident)
    TUPLE_CODECS = tuple_codes()

    metric = args.metric
    target_arr = args.target_arr
    target_tol = args.target_tol
    db_file_name = args.db_file_name
    only_perform_missing_encodes = args.only_perform_missing_encodes
    show_and_recode_compress_args(LOGGER, args)
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
    create_table_if_needed(LOGGER, CONNECTION, only_perform_missing_encodes)

    pool = multiprocessing.Pool(processes=args.num_process, initializer=initialize_worker)
    images = set(listdir_full_path('/code/images'))
    if len(images) <= 0:
        LOGGER.error(" no source files in ./images.")
        exit(1)

    results = list()

    for target in target_arr:
        for num, image in enumerate(images):
            width, height, depth = get_dimensions(LOGGER, image)
            LOGGER.info(
                "[" + str(num) + "] Source image: " + image + " {" + width + "x" + height + "} bit-depth: " + depth)

            for codec in TUPLE_CODECS:
                LOGGER.debug(" ")
                skip_encode = False
                if only_perform_missing_encodes:
                    unique_id = make_my_tuple(LOGGER, image, width, height, codec.name, metric, target,
                                              codec.subsampling)
                    skip_encode = does_entry_exist(LOGGER, CONNECTION, unique_id)

                if not skip_encode:
                    results.append(
                        (pool.apply_async(bisection,
                                          args=(codec.inverse, codec.param_start, codec.param_end, codec.ab_tol,
                                                metric, target, target_tol, codec.name, image, width, height,
                                                codec.subsampling),
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
            for target in target_arr:
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
            for target in target_arr:
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
