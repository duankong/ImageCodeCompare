import os
from collections import Counter
import logging
import uuid
import multiprocessing
import threading
import sqlite3
import numpy as np

from utils.m_data_class import ImageData
# utils
from utils.u_mysql_execute import create_table_if_needed, does_entry_exist, get_insert_command
from utils.u_logging_setup import setup_logging
from utils.u_result_easy_show import result_video_show, result_lossless_show
from utils.u_utils_common import make_my_tuple_video, mkdir_p
# config
from Config.config_compress import args_compress_video_config
from Config.config_utils import video_tuple_choice
from utils.a_video_lossless_8bit_cmd import f_video_lossless_8bit
from utils.a_video_lossy_8bit_cmd import f_video_lossly_8bit

args = args_compress_video_config()
TOTAL_BYTES = Counter()
TOTAL_METRIC = Counter()
TOTAL_ERRORS = Counter()

LOGGER = logging.getLogger('video.compression')
CONNECTION = None
WORK_DIR = args.work_dir


def update_stats(results):
    """ callback function called when a worker process finishes an encoding job with target quality value
    """
    channels = 1
    param, quality, encoded_file, file_size_bytes, codec_status, im_status = results
    LOGGER.info('<<' + codec_status['codec'].upper() + '>>' + " Param " + str(param) + ", quality "
                + repr(quality) + ", encoded_file: " + encoded_file
                + " size: " + str(file_size_bytes) + " bytes")
    TOTAL_BYTES[codec_status['codec'] + im_status['subsampling'] + codec_status['metric'] + str(
        codec_status['target'])] += os.path.getsize(encoded_file)
    TOTAL_METRIC[codec_status['codec'] + im_status['subsampling'] + codec_status['metric'] + str(
        codec_status['target'])] += quality[codec_status['metric']]

    source_file_size = int(im_status['width']) * int(im_status['height']) * im_status['frames'] * channels * im_status[
        'depth'] / 8

    bpp = os.path.getsize(encoded_file) * 8.0 / (
            int(im_status['width']) * int(im_status['height']) * im_status['frames'] * channels)

    compress_rate = source_file_size / os.path.getsize(encoded_file)

    print(
        "source_file_size ={} Bytes BPP = {} Compress_rate = {} frames ={} ".format(source_file_size, bpp,
                                                                                    compress_rate,
                                                                                    im_status['frames']))
    try:
        # noinspection PyUnresolvedReferences
        CONNECTION.execute(get_insert_command(), (
            codec_status['tuple_minus_uuid'], im_status['source_image'], im_status['width'],
            im_status['height'],
            im_status['depth'],
            codec_status['codec'], param, im_status['temp_folder'], codec_status['metric'], codec_status['target'],
            quality['vmaf'], quality['ssim'], quality['ms_ssim'], quality['vif'],
            quality['mse_y'], quality['mse_u'], quality['mse_v'],
            quality['mse_avg'],
            quality['psnr_y'], quality['psnr_u'], quality['psnr_v'],
            quality['psnr_avg'],
            quality['adm2'],
            im_status['subsampling'], file_size_bytes, encoded_file,
            bpp, compress_rate, im_status['frames'], source_file_size
        ))
        # noinspection PyUnresolvedReferences
        CONNECTION.commit()
    except Exception:
        LOGGER.error("[ update_stats ] ERROR")
        exit(0)
    else:
        pass
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


def bisection(inverse, a, b, ab_tol, metric, target, target_tol, codec, yuv_file, width, height, real_frames, depth,
              subsampling, model, param):
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
                                                 subsampling, param,
                                                 uuid=temp_uuid)
    tuple_minus_uuid = make_my_tuple_video(LOGGER, yuv_file, width, height, real_frames, codec, metric, target,
                                           subsampling, param)
    mkdir_p(temp_folder)

    image_status = dict(
        [('source_image', yuv_file),
         ('width', width), ('height', height), ('depth', 8), ('frames', real_frames),
         ('subsampling', subsampling), ('temp_folder', temp_folder)])

    compress_status = dict(
        [('metric', metric), ('target', target), ('codec', codec), ('tuple_minus_uuid', tuple_minus_uuid)])

    yuv_status = dict(
        [('yuv_file', yuv_file), ('width', width), ('height', height), ('depth', depth), ('frames', real_frames),
         ('subsampling', subsampling)])

    LOGGER.debug(repr((multiprocessing.current_process(), temp_folder,
                       inverse, a, b, ab_tol, metric, target, target_tol, codec, yuv_file, width, height, real_frames,
                       subsampling)))

    # run
    last_c, quality, encoded_file = None, None, None
    if model == 'customize':
        c = (a + b) / 2
        last_c = c
        while (b - a) > ab_tol:

            quality, encoded_file = f_video_lossly_8bit(LOGGER, codec, yuv_status, c, temp_folder)
            last_c = c

            if abs(quality[metric] - target) < target_tol:
                return last_c, quality, encoded_file, os.path.getsize(encoded_file), compress_status, image_status
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
    elif model == 'lossless':
        last_c = '0'
        quality, encoded_file = f_video_lossless_8bit(LOGGER, codec, yuv_status, last_c, temp_folder)
    elif model == 'auto':
        last_c = param
        quality, encoded_file = f_video_lossly_8bit(LOGGER, codec, yuv_status, last_c, temp_folder)
    else:
        LOGGER.error("[bisection] Not support mode in {}".format(model))
        exit(0)
    return last_c, quality, encoded_file, os.path.getsize(encoded_file), compress_status, image_status


def func(pool, data, tuple_codes, only_perform_missing_encodes, target, metric, results, target_tol, model):
    # for num in range(int(data.image_nums / frames) + 1):
    for num in range(1):
        if num == int(data.image_nums / data.max_frames):
            real_frames = data.image_nums % data.max_frames
        else:
            real_frames = data.max_frames
        yuv_files = os.path.join(data.yuv_dir, "IMG_{}bit_{:04}".format(data.depth, num))

        LOGGER.info(
            '[{}] Source yuv file: {} {}x{}x{:<3} bit-depth: {}'.format(num, yuv_files, data.width, data.height,
                                                                        real_frames,
                                                                        data.depth))
        for codec in tuple_codes:
            LOGGER.debug(" ")
            skip_encode = False
            if only_perform_missing_encodes:
                unique_id = make_my_tuple_video(LOGGER, yuv_files, data.width, data.height, real_frames, codec.name,
                                                metric,
                                                target,
                                                codec.subsampling, 0)
                skip_encode = does_entry_exist(LOGGER, CONNECTION, unique_id)
            if not skip_encode:
                if model in ['lossless', 'customize']:
                    results.append(
                        (pool.apply_async(bisection,
                                          args=(codec.inverse, codec.param_start, codec.param_end, codec.ab_tol,
                                                metric, target, target_tol, codec.name, yuv_files, data.width,
                                                data.height,
                                                real_frames, data.depth,
                                                codec.subsampling, model, 0),
                                          callback=update_stats,
                                          error_callback=error_function),
                         codec.name,
                         codec.subsampling))
                elif model == 'auto':
                    for param in np.linspace(codec.param_start, codec.param_end, 5):
                        results.append(
                            (pool.apply_async(bisection,
                                              args=(codec.inverse, codec.param_start, codec.param_end, codec.ab_tol,
                                                    metric, target, target_tol, codec.name, yuv_files, data.width,
                                                    data.height,
                                                    real_frames, data.depth,
                                                    codec.subsampling, model, param),
                                              callback=update_stats,
                                              error_callback=error_function),
                             codec.name,
                             codec.subsampling))
                else:
                    LOGGER.error("[func] Not support mode in {}".format(args.func_choice))
        LOGGER.info('-----------------------------------------------------------------------------------------')


def main_func():
    # ===================================     Prepare YUVs     =================================== #
    data = ImageData(args.image_path)
    metric = args.metric
    target_arr = args.target_arr
    target_tol = args.target_tol
    db_file_name = os.path.join(WORK_DIR, args.db_file_name)
    num_process = args.num_process
    frames = args.yuv_frames
    model = args.func_choice

    data.init_yuv_info(args.batch_image_dir, args.yuv_dir, args.yuv_frames)

    if args.prepare_yuv:
        data.yuv_prepare(num_process=num_process)

    # =====================================   only_perform_missing_encodes   ===================================== #

    setup_logging(LOGGER=LOGGER, worker=False, worker_id=multiprocessing.current_process().ident)
    only_perform_missing_encodes = args.only_perform_missing_encodes

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
    # ===================================     Run     =================================== #
    pool = multiprocessing.Pool(processes=args.num_process, initializer=initialize_worker)
    results = list()
    tuple_codecs = video_tuple_choice(LOGGER, '8', args.func_choice)
    if model == 'customize':
        for target in target_arr:
            func(pool, data, tuple_codecs, only_perform_missing_encodes, target, metric, results, target_tol, model)
        result_video_show(LOGGER, data.images, results, TOTAL_ERRORS, tuple_codecs, TOTAL_METRIC,
                          TOTAL_BYTES, only_perform_missing_encodes, frames, metric, target_arr)
    elif model == 'lossless':
        metric = 'psnr_avg'
        target = 0
        func(pool, data, tuple_codecs, only_perform_missing_encodes, target, metric, results, target_tol, model)
        result_lossless_show(LOGGER, data.images, results, TOTAL_ERRORS, tuple_codecs, TOTAL_METRIC,
                             TOTAL_BYTES, only_perform_missing_encodes, frames, metric, target)
    elif model == 'auto':
        metric = 'psnr_avg'
        target = 0
        func(pool, data, tuple_codecs, only_perform_missing_encodes, target, metric, results, target_tol, model)
    else:
        LOGGER.error("[Config] Not support mode in {}".format(args.func_choice))
        exit(0)
    # ===================================     END     =================================== #
    pool.close()
    pool.join()
    CONNECTION.close()
    LOGGER.info("\n\n")
    LOGGER.info("[*] --------------------------- Done --------------------------- [*]")
    logging.shutdown()
    os.system("stty echo")
    return


if __name__ == '__main__':
    main_func()
