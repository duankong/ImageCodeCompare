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
from collections import Counter
import logging
import uuid
import multiprocessing
import ntpath
import threading
import sqlite3

from utils.UtilsCommon import make_my_tuple, mkdir_p
from utils.MysqlFun import get_insert_command, create_table_if_needed, does_entry_exist
# change
from utils.sub_work import setup_logging
from Config.config_compress import args_8bit_image_compress_config
from utils.result_easy_show import show_image_lossy_result, show_image_lossless_result
from Config.image_8bit_config import tuple_codes, lossless_tuple_codes

from utils.a_image_lossy_8bit_cmd import f_image_lossy_8bit
from utils.a_image_lossless_8bit_cmd import f_image_lossless_8bit

from utils.Data_Prepare import ImageData

TOTAL_BYTES = Counter()
TOTAL_METRIC = Counter()
TOTAL_ERRORS = Counter()

LOGGER = logging.getLogger('image.compression')
CONNECTION = None
args = args_8bit_image_compress_config()
WORK_DIR = args.work_dir


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

    if args.lossless == False:
        c = (a + b) / 2
        last_c = c
        while (b - a) > ab_tol:

            quality, encoded_file = f_image_lossy_8bit(LOGGER, c, codec, image, width, height, temp_folder, subsampling)
            last_c = c

            if abs(quality[metric] - target) < target_tol:
                return (
                    last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file), subsampling,
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
    else:
        last_c = '0'
        quality, encoded_file = f_image_lossless_8bit(LOGGER, image, width, height, temp_folder, codec, subsampling,
                                                      last_c)

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


def func(data, pool, TUPLE_CODECS, only_perform_missing_encodes, results, metric, target, target_tol):
    width, height, depth = data.width, data.height, data.depth
    for num, image in enumerate(data.images):
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


def main():
    """ create a pool of worker processes and submit encoding jobs, collect results and exit
    """
    # =========================================   Init DATA and Parameter   ========================================= #
    LOGGER.info(
        'started main, current thread ID %s %s %s', multiprocessing.current_process(),
        multiprocessing.current_process().pid,
        threading.current_thread().ident)
    setup_logging(LOGGER=LOGGER, worker=False, worker_id=multiprocessing.current_process().ident)
    data = ImageData(args.image_path)
    metric = args.metric
    target_arr = args.target_arr
    target_tol = args.target_tol
    db_file_name = os.path.join(WORK_DIR, args.db_file_name)

    # =======================================   only_perform_missing_encodes   ======================================= #
    only_perform_missing_encodes = args.only_perform_missing_encodes

    if only_perform_missing_encodes:
        if os.path.isfile(db_file_name):
            LOGGER.info("Will add missing entries to file " + db_file_name)
        else:
            LOGGER.error("only_perform_missing_encodes is True but db file " + db_file_name + " does not exist.")
            exit(1)

    global CONNECTION
    CONNECTION = sqlite3.connect(db_file_name, check_same_thread=False)
    create_table_if_needed(LOGGER, CONNECTION, only_perform_missing_encodes)

    # ===================================================   RUN   =================================================== #
    pool = multiprocessing.Pool(processes=args.num_process, initializer=initialize_worker)
    results = list()
    if args.lossless == False:
        TUPLE_CODECS = tuple_codes()
        for target in target_arr:
            func(data, pool, TUPLE_CODECS, only_perform_missing_encodes, results, metric, target, target_tol)
        show_image_lossy_result(results, only_perform_missing_encodes, LOGGER, TOTAL_ERRORS, TOTAL_METRIC, TOTAL_BYTES,
                                TUPLE_CODECS, data, target_arr, metric)
    else:
        metric = 'psnr_avg'
        target = 0

        TUPLE_CODECS = lossless_tuple_codes()
        func(data, pool, TUPLE_CODECS, only_perform_missing_encodes, results, metric, target, target_tol)
        show_image_lossless_result(results, only_perform_missing_encodes, LOGGER, TOTAL_ERRORS, TOTAL_METRIC,
                                   TOTAL_BYTES,
                                   TUPLE_CODECS, target, metric, data.image_nums)

    pool.close()
    pool.join()
    CONNECTION.close()
    LOGGER.info("\n\n[*] --------------------------- Done --------------------------- [*]")
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
