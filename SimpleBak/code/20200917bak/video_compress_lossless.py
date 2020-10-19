import os
from collections import Counter
import logging
import uuid
import multiprocessing
import ntpath
import threading
import sqlite3
from utils.PNG2YUV import yuv_prepare, listdir_full_path, get_dimensions

from utils.MetricCaculate import compute_metrics
from utils.FormatHelp import format_adress
from utils.cmd_root_path import video_tuple_codes
from utils.run_cmd import run_program, my_exec
from utils.MysqlFun import create_table_if_needed, does_entry_exist, get_insert_command
from utils.UtilsCommon import get_filename_with_temp_folder, make_my_tuple_video, float_to_int, mkdir_p, \
    listdir_full_path, \
    setup_logging

from Config.config_compress import args_compress_video_config, show_and_recode_compress_args

args = args_compress_video_config()
TOTAL_BYTES = Counter()
TOTAL_METRIC = Counter()
TOTAL_ERRORS = Counter()

LOGGER = logging.getLogger('video.compression')
CONNECTION = None
WORK_DIR = args.work_dir
FORM = format_adress()


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


def f(codec, yuv_status, param, temp_folder):
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

    # 1 HEVC
    if codec == 'hevc' and subsampling in ['420', '444']:

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['{}/TAppEncoderHighBitDepthStatic'.format(FORM.hevc),
               '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_high_throughput_rext.cfg',
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
        cmd = ['{}/TAppDecoderHighBitDepthStatic'.format(FORM.hevc),
               '--OutputBitDepthC={}'.format(depth),
               '--OutputBitDepth={}'.format(depth),
               '-b', encoded_file,
               '-o', decoded_yuv]
        my_exec(LOGGER, cmd)
    # 2 AVIF
    elif codec in ['avif-mse', 'avif-ssim'] and subsampling in ['420', '444']:
        param = float_to_int(param)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.ivf')
        cmd = ['aomenc', '--i420' if subsampling == '420' else '--i444',
               '--width={}'.format(width), '--height={}'.format(height),
               '--input-bit-depth={}'.format(depth), '--bit-depth={}'.format(depth),
               '--cq-level={}'.format(param), '--min-q={}'.format(param), '--max-q={}'.format(param),
               '--ivf',
               '--good',
               '--passes=2',
               '--cpu-used=8',
               '--end-usage=cbr',
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
    # 3 AVS3
    elif codec in ['avs3'] and subsampling in ['420', '444']:
        param = float_to_int(param)
        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avs')
        #     1 ./uavs3enc -i ～/Document/VideoCoding/akiyo_qcif.yuv -w 176 -h 144 --fps_num 10 --fps_den 10 -d 8 -o ~/Document/VideoCoding/output.avs3
        cmd = ['{}/uavs3enc'.format(FORM.avs3enc),
               '-i', source_yuv, '-w', str(width), '-h', str(height), '-d', str(depth), '-f', str(frames),
               '-q', param,
               '--fps_num', '30',
               '--fps_den', '1',
               '--frm_threads', '{}'.format(depth),
               '-o', encoded_file]
        run_program(LOGGER, cmd)

        cmd = ['{}/uavs3dec'.format(FORM.avs3dec),
               '-i', encoded_file,
               '-t', '8',
               '-f', str(frames),
               '-o', decoded_yuv]
        run_program(LOGGER, cmd)

    else:
        raise RuntimeError('Unsupported codec and subsampling ' + codec + ' / ' + subsampling)

    stats = compute_metrics(LOGGER, source_yuv, decoded_yuv, width, height, depth, temp_folder, subsampling)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)
    LOGGER.debug(
        "Encoding  {} with {:<13} param={:<8} subsampling={}|| "
        "psnr_avg={:<8} ssim={:<8}  vmaf={:<8} || "
        "size={} ".format(yuvsource, codec,
                          param[0:8],
                          subsampling,
                          str(stats['psnr_avg'])[0:8],
                          stats['ssim'],
                          stats['vmaf'],
                          stats['file_size_bytes']))
    return stats, encoded_file


def bisection(inverse, a, b, ab_tol, metric, target, target_tol, codec, yuv_file, width, height, real_frames, depth,
              subsampling):
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
    :param yuv_file: source image
    :param width: width of source image and target encode
    :param height: height of source image and target encode
    :param subsampling: color subsampling
    :return:
    """
    temp_uuid = str(uuid.uuid4())
    temp_folder = WORK_DIR + make_my_tuple_video(LOGGER, yuv_file, width, height, real_frames, codec, metric, target,
                                                 subsampling,
                                                 uuid=temp_uuid)
    tuple_minus_uuid = make_my_tuple_video(LOGGER, yuv_file, width, height, real_frames, codec, metric, target,
                                           subsampling)
    mkdir_p(temp_folder)

    yuv_status = dict(
        [('yuv_file', yuv_file), ('width', width), ('height', height), ('depth', depth), ('frames', real_frames),
         ('subsampling', subsampling)])
    LOGGER.debug(repr((multiprocessing.current_process(), temp_folder,
                       inverse, a, b, ab_tol, metric, target, target_tol, codec, yuv_file, width, height, real_frames,
                       subsampling)))

    c = (a + b) / 2
    last_c = c
    while (b - a) > ab_tol:

        quality, encoded_file = f(codec, yuv_status, c, temp_folder)
        last_c = c

        if abs(quality[metric] - target) < target_tol:
            return (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file), subsampling,
                    tuple_minus_uuid, ntpath.basename(yuv_file), width, height, temp_folder)
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
            tuple_minus_uuid, ntpath.basename(yuv_file), width, height, temp_folder)


def main_func():
    # ===================================     Prepare YUVs     =================================== #
    souceimages = args.souceimages
    batch_image_dir = args.batch_image_dir
    yuv_dir = args.yuv_dir
    num_process = args.num_process
    frames = args.yuv_frames

    batch_image_dir = '/code/AAA/batch'
    yuv_dir = '/code/AAA/yuvs'

    if args.prepare_yuv:
        yuv_prepare(souceimages=souceimages, batch_image_dir=batch_image_dir, yuv_dir=yuv_dir, num_process=num_process,
                    step=frames)

    # ===================================     Init      =================================== #
    setup_logging(LOGGER=LOGGER, worker=False, worker_id=multiprocessing.current_process().ident)
    TUPLE_CODECS = video_tuple_codes()
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
    # ===================================     Run     =================================== #
    images = list(set(listdir_full_path(souceimages)))
    width, height, depth = get_dimensions(images[0])
    real_frames = frames
    if len(images) <= 0:
        LOGGER.error(" no source files in ./images.")
        exit(1)

    results = list()
    for target in target_arr:
        # for num in range(int(len(images) / frames) + 1):
        for num in range(1):
            if num == int(len(images) / frames):
                real_frames = len(images) % frames
            yuv_files = os.path.join(yuv_dir, "IMG_{}bit_{:04}".format(depth, num))

            LOGGER.info(
                '[{}] Source yuv file: {} {}x{}x{:<3} bit-depth:{}'.format(num, yuv_files, width, height, real_frames,
                                                                           depth))
            for codec in TUPLE_CODECS:
                LOGGER.debug(" ")
                skip_encode = False
                if only_perform_missing_encodes:
                    unique_id = make_my_tuple_video(LOGGER, yuv_files, width, height, real_frames, codec.name, metric,
                                                    target,
                                                    codec.subsampling)
                    skip_encode = does_entry_exist(LOGGER, CONNECTION, unique_id)

                if not skip_encode:
                    results.append(
                        (pool.apply_async(bisection,
                                          args=(codec.inverse, codec.param_start, codec.param_end, codec.ab_tol,
                                                metric, target, target_tol, codec.name, yuv_files, width, height,
                                                real_frames, depth,
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
                                                      int(len(images) / frames) + 1)))

    CONNECTION.close()
    LOGGER.info("\n\n")
    LOGGER.info("[*] --------------------------- Done --------------------------- [*]")
    logging.shutdown()
    os.system("stty echo")
    return


if __name__ == '__main__':
    main_func()
