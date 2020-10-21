import os
from collections import Counter
import logging
import uuid
import multiprocessing
import ntpath
import threading
import sqlite3

from utils.MysqlFun import create_table_if_needed, does_entry_exist, get_insert_command
from utils.UtilsCommon import make_my_tuple_video, mkdir_p

# change
from utils.Data_Prepare import ImageData
from Config.config_compress import args_compress_video_config
from Config.video_8bit_config import video_tuple_codes, video_tuple_lossless_codes
from utils.a_video_lossless_8bit_cmd import f_video_lossless_8bit
from utils.a_video_lossy_8bit_cmd import f_video_lossly_8bit
from utils.sub_work import setup_logging
from utils.result_easy_show import result_video_show, result_lossless_show

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

    if args.lossless == False:
        c = (a + b) / 2
        last_c = c
        while (b - a) > ab_tol:

            quality, encoded_file = f_video_lossly_8bit(LOGGER, codec, yuv_status, c, temp_folder)
            last_c = c

            if abs(quality[metric] - target) < target_tol:
                return (
                    last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file), subsampling,
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
    else:
        last_c = '0'
        quality, encoded_file = f_video_lossless_8bit(LOGGER, codec, yuv_status, last_c, temp_folder)

    return (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file), subsampling,
            tuple_minus_uuid, ntpath.basename(yuv_file), width, height, temp_folder)


def func(pool, data, TUPLE_CODECS, only_perform_missing_encodes, target, metric, results, target_tol):
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
        for codec in TUPLE_CODECS:
            LOGGER.debug(" ")
            skip_encode = False
            if only_perform_missing_encodes:
                unique_id = make_my_tuple_video(LOGGER, yuv_files, data.width, data.height, real_frames, codec.name,
                                                metric,
                                                target,
                                                codec.subsampling)
                skip_encode = does_entry_exist(LOGGER, CONNECTION, unique_id)

            if not skip_encode:
                results.append(
                    (pool.apply_async(bisection,
                                      args=(codec.inverse, codec.param_start, codec.param_end, codec.ab_tol,
                                            metric, target, target_tol, codec.name, yuv_files, data.width, data.height,
                                            real_frames, data.depth,
                                            codec.subsampling),
                                      callback=update_stats,
                                      error_callback=error_function),
                     codec.name,
                     codec.subsampling))
        LOGGER.info('-----------------------------------------------------------------------------------------')


def main_func():
    # ===================================     Prepare YUVs     =================================== #
    data = ImageData(args.image_path)
    metric = args.metric
    target_arr = args.target_arr
    target_tol = args.target_tol
    db_file_name = os.path.join(WORK_DIR,args.db_file_name)
    num_process = args.num_process
    frames = args.yuv_frames

    data.init_yuv_info(args.batch_image_dir, args.yuv_dir, args.yuv_frames)

    if args.prepare_yuv == True:
        data.yuv_prepare(num_process=num_process)

    # =========================================   only_perform_missing_encodes   ========================================= #

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
    if args.lossless == False:
        TUPLE_CODECS = video_tuple_codes()
        for target in target_arr:
            func(pool, data, TUPLE_CODECS, only_perform_missing_encodes, target, metric, results, target_tol)
        result_video_show(LOGGER, data.images, results, TOTAL_ERRORS, TUPLE_CODECS, TOTAL_METRIC,
                          TOTAL_BYTES, only_perform_missing_encodes, frames, metric, target_arr)
    elif args.lossless == True:
        TUPLE_CODECS = video_tuple_lossless_codes()
        target = 0
        func(pool, data, TUPLE_CODECS, only_perform_missing_encodes, target, metric, results, target_tol)
        result_lossless_show(LOGGER, data.images, results, TOTAL_ERRORS, TUPLE_CODECS, TOTAL_METRIC,
                             TOTAL_BYTES, only_perform_missing_encodes, frames, metric, target)
    else:
        LOGGER.debug('args.lossless ERROR !!')
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