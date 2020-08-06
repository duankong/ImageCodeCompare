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
import sys
import subprocess
import json
import glob
from collections import Counter
import logging
from collections import namedtuple
import datetime
import uuid
import multiprocessing
import ntpath
import threading
import sqlite3
from skimage import io, measure

from utils import *

__author__ = "Aditya Mavlankar"
__copyright__ = "Copyright 2019-2020, Netflix, Inc."
__credits__ = ["Kyle Swanson", "Jan de Cock", "Marjan Parsa"]
__license__ = "Apache License, Version 2.0"
__version__ = "0.1"
__maintainer__ = "Aditya Mavlankar"
__email__ = "amavlankar@netflix.com"
__status__ = "Development"

TOTAL_BYTES = Counter()
TOTAL_METRIC = Counter()
TOTAL_ERRORS = Counter()

LOGGER = logging.getLogger('image.compression')
CONNECTION = None
TUPLE_CODECS = tuple_codes()
WORK_DIR = "/image_test/"


def setup_logging(worker, worker_id):
    """
    set up logging for the process calling this function.
    :param worker: True means it is a worker process from the pool. False means it is the main process.
    :param worker_id: unique ID identifying the process
    :return:
    """
    for h in list(LOGGER.handlers):
        LOGGER.removeHandler(h)
    now = datetime.datetime.now()
    formatter = logging.Formatter('%(asctime)s - %(threadName)s - %(levelname)s - %(message)s')
    name = "compression_results_"
    if worker:
        name += "worker_"
        global CONNECTION
        CONNECTION = None
    name += str(worker_id) + '_' + now.isoformat() + '.txt'
    file_log_handler = logging.FileHandler(name)
    file_log_handler.setFormatter(formatter)
    LOGGER.addHandler(file_log_handler)

    console_log_handler = logging.StreamHandler()
    console_log_handler.setFormatter(formatter)
    LOGGER.addHandler(console_log_handler)

    LOGGER.setLevel('DEBUG')


def run_program(*args, **kwargs):
    """ run command using subprocess.check_output, collect stdout and stderr together and return
    """
    kwargs.setdefault("stderr", subprocess.STDOUT)
    kwargs.setdefault("shell", True)

    try:
        output = subprocess.check_output(" ".join(*args), **kwargs)
        return decode(output)

    except subprocess.CalledProcessError as e:
        LOGGER.error("***** ATTENTION : subprocess call crashed: %s\n%s", args, e.output)
        raise


def get_dimensions(image):
    """ given a source image, return dimensions and bit-depth
    """
    dimension_cmd = ["identify", '-format', '%w,%h,%z', image]
    res = run_program(dimension_cmd)
    res_split = res.split('\n')
    width, height, depth = res_split[len(res_split) - 1].split(
        ',')  # assuming last line has the goodies; warnings, if any, appear before
    return width, height, depth


def my_exec(cmd):
    """ helper to choose method for running commands
    """
    return run_program(cmd)


def convert_format(image, temp_folder, format):
    image_convert = get_filename_with_temp_folder(temp_folder, "image_convert.{}".format(str(format)))
    cmd = ['convert', image, image_convert]
    my_exec(cmd)
    return image_convert


def kakadu_encode_helper(image, temp_folder, param):
    # convert
    image_convert = convert_format(image, temp_folder, "pgm")
    # kakadu encode
    param = str(param)
    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.jp2')
    cmd = ['/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602/kdu_compress', '-quiet', '-i',
           image_convert, '-o', ntpath.basename(encoded_file), '-rate', param]
    run_program(cmd, cwd=temp_folder)
    # kakadu decode
    decoded_file = get_filename_with_temp_folder(temp_folder, 'kakadu_decoded.bmp')

    for filePath in glob.glob(get_filename_with_temp_folder(temp_folder, 'kakadu_decoded*.bmp')):
        try:
            os.remove(filePath)
        except:
            LOGGER.error("Error while deleting file : " + filePath)
    cmd = ['/tools/kakadu/KDU805_Demo_Apps_for_Linux-x86-64_200602/kdu_expand', '-quiet', '-i', encoded_file,
           '-o', decoded_file]
    my_exec(cmd)
    decoded_bmp_files = glob.glob(get_filename_with_temp_folder(temp_folder, 'kakadu_decoded*.bmp'))
    assert len(decoded_bmp_files) == 1
    decoded_file = decoded_bmp_files[0]
    return encoded_file, decoded_file


def jpeg_encode_helper(image, temp_folder, param):
    # convert
    source_image_convert = get_filename_with_temp_folder(temp_folder, 'source.pgm')
    cmd = ['convert', image, source_image_convert]
    my_exec(cmd)
    # jpeg encode
    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.jpg')
    cmd = ['/tools/jpeg-9d/cjpeg', '-dct int', '-quality', str(param), '-outfile', encoded_file,
           source_image_convert]
    my_exec(cmd)
    # jpeg decoder
    decoded_file = get_filename_with_temp_folder(temp_folder, 'jpeg_decode.bmp')
    cmd = ['/tools/jpeg-9d/djpeg', '-dct int', '-bmp', '-outfile', decoded_file, encoded_file]
    my_exec(cmd)
    return encoded_file, decoded_file


def webp_encod_helper(image, temp_folder, param):
    # webp encode
    encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.webp')
    cmd = ['/tools/libwebp-1.1.0-linux-x86-64/bin/cwebp', '-noalpha', '-print_ssim', '-m', '6', '-q', str(param), image,
           '-o', encoded_file]
    my_exec(cmd)
    # webp decoder
    decoded_file = get_filename_with_temp_folder(temp_folder, 'webp_decode.png')
    cmd = ['/tools/libwebp-1.1.0-linux-x86-64/bin/dwebp', '-quiet', encoded_file, '-o', decoded_file]
    my_exec(cmd)

    # convert
    source_image_convert = get_filename_with_temp_folder(temp_folder, 'webp_decode_convert.png')
    cmd = ['convert', image, '-colorspace RGB', '-colorspace Gray', source_image_convert]
    my_exec(cmd)
    return encoded_file, source_image_convert


def f(param, codec, image, width, height, temp_folder):
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
    :return:
    """
    subsampling = '420'

    if not isinstance(param, str):
        param = str(param)

    LOGGER.debug("Encoding image " + image + " with codec " + codec + " for param " + param)

    cmd = ['cp', image, temp_folder]
    my_exec(cmd)

    source_image = get_filename_with_temp_folder(temp_folder, ntpath.basename(image))

    encoded_file = ""
    decoded_file = ""

    if codec in ['jpeg']:

        encoded_file, decoded_file = jpeg_encode_helper(image, temp_folder, param)

    elif codec == "webp":
        encoded_file, decoded_file = webp_encod_helper(image, temp_folder, param)

    elif codec in ['kakadu']:

        encoded_file, decoded_file = kakadu_encode_helper(image, temp_folder, param)

    elif codec == 'hevc' and subsampling in ['420', '444u']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        my_exec(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['/tools/HM-16.20+SCM-8.8/bin/TAppEncoderStatic',
               '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_main_rext.cfg', '-f', '1', '-fr', '1', '-q', param,
               '-wdt', width, '-hgt', height, '--InputChromaFormat=420', '--ConformanceWindowMode=1',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(cmd)

        cmd = ['/tools/HM-16.20+SCM-8.8/bin/TAppDecoderStatic', '-b', encoded_file, '-d', '8', '-o', decoded_yuv]
        my_exec(cmd)

        if subsampling == '444u':
            source_yuv, decoded_yuv = convert_420_to_444_source_and_decoded(source_yuv, decoded_yuv, image, width,
                                                                            height, subsampling)

    elif codec == 'hevc' and subsampling == '444':
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        my_exec(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.hevc')
        cmd = ['/tools/HM-16.20+SCM-8.8/bin/TAppEncoderStatic',
               '-c', '/tools/HM-16.20+SCM-8.8/cfg/encoder_intra_main_rext.cfg', '-f', '1', '-fr', '1', '-q', param,
               '-wdt', width, '-hgt', height, '--InputChromaFormat=444', '--ConformanceWindowMode=1',
               '-i', source_yuv, '-b', encoded_file, '-o', '/dev/null']
        my_exec(cmd)

        cmd = ['/tools/HM-16.20+SCM-8.8/bin/TAppDecoderStatic', '-b', encoded_file, '-d', '8', '-o', decoded_yuv]
        my_exec(cmd)

    elif codec in ['avif-mse', 'avif-ssim'] and subsampling in ['420', '444u']:
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        my_exec(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avif')
        cmd = ['aomenc', '--i420', '--width={}'.format(width), '--height={}'.format(height), '--ivf', '--cpu-used=1',
               '--end-usage=q', '--cq-level={}'.format(param), '--min-q={}'.format(param), '--max-q={}'.format(param),
               '--passes=2', '--input-bit-depth={}'.format(8), '--bit-depth={}'.format(8), '--lag-in-frames=0',
               '--frame-boost=0', '--disable-warning-prompt',
               '--output={}'.format(encoded_file), source_yuv]
        if codec == 'avif-ssim':
            cmd[-2:-2] = ['--tune=ssim']
        my_exec(cmd)

        cmd = ['aomdec', '--i420', '-o', decoded_yuv, encoded_file]
        my_exec(cmd)

        if subsampling == '444u':
            source_yuv, decoded_yuv = convert_420_to_444_source_and_decoded(source_yuv, decoded_yuv, image, width,
                                                                            height, subsampling)

    elif codec in ['avif-mse', 'avif-ssim'] and subsampling == '444':
        cmd = ['ffmpeg', '-y', '-i', image, '-pix_fmt', get_pixel_format_for_encoding(subsampling), source_yuv]
        my_exec(cmd)

        encoded_file = get_filename_with_temp_folder(temp_folder, 'temp.avif')
        cmd = ['aomenc', '--i444', '--width={}'.format(width), '--height={}'.format(height), '--ivf', '--cpu-used=1',
               '--end-usage=q', '--cq-level={}'.format(param), '--min-q={}'.format(param), '--max-q={}'.format(param),
               '--passes=2', '--input-bit-depth={}'.format(8), '--bit-depth={}'.format(8), '--lag-in-frames=0',
               '--frame-boost=0', '--disable-warning-prompt',
               '--output={}'.format(encoded_file), source_yuv]
        if codec == 'avif-ssim':
            cmd[-2:-2] = ['--tune=ssim']
        my_exec(cmd)

        cmd = ['aomdec', '--rawvideo', '-o', decoded_yuv, encoded_file]
        my_exec(cmd)

    else:
        raise RuntimeError("Unsupported codec : {} ".format(codec))
    stats = compute_metrics(source_image, decoded_file, width, height, temp_folder)
    stats['file_size_bytes'] = os.path.getsize(encoded_file)
    return stats, encoded_file


def make_my_tuple(image, width, height, codec, metric, target, uuid=None):
    """ make unique tuple for unique directory, primary key in DB, etc.
    """
    my_tuple = '{image}_{width}x{height}_{codec}_{metric}_{target}_' \
        .format(image=ntpath.basename(image), width=width, height=height, codec=codec,
                metric=metric, target=target)
    if uuid is not None:
        my_tuple = my_tuple + uuid
    if len(my_tuple) > 255:  # limits due to max dir name or file name length on UNIX
        LOGGER.error("ERROR : Tuple too long : " + my_tuple)
    assert len(my_tuple) < 256
    return my_tuple


def bisection(inverse, a, b, ab_tol, metric, target, target_tol, codec, image, width, height):
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
    temp_folder = WORK_DIR + make_my_tuple(image, width, height, codec, metric, target, temp_uuid)
    print("******* {}".format(temp_folder))
    tuple_minus_uuid = make_my_tuple(image, width, height, codec, metric, target)
    mkdir_p(temp_folder)
    LOGGER.debug(repr((multiprocessing.current_process(), temp_folder,
                       inverse, a, b, ab_tol, metric, target, target_tol, codec, image, width, height)))

    c = (a + b) / 2
    if isinstance(ab_tol, int):
        c = int(c)
    last_c = c
    while (b - a) > ab_tol:
        quality, encoded_file = f(c, codec, image, width, height, temp_folder)
        last_c = c

        if abs(quality[metric] - target) < target_tol:
            return (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file),
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
            if isinstance(ab_tol, int):
                c = int(c)

    return (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file),
            tuple_minus_uuid, ntpath.basename(image), width, height, temp_folder)


def update_stats(results, verbose=0):
    """ callback function called when a worker process finishes an encoding job with target quality value
    """
    c, codec, metric, target, quality, encoded_file, file_size_bytes, \
    tuple_minus_uuid, source_image, width, height, temp_folder = results

    # (last_c, codec, metric, target, quality, encoded_file, os.path.getsize(encoded_file),
    #  tuple_minus_uuid, ntpath.basename(image), width, height, temp_folder)

    LOGGER.info('<<' + codec.upper() + '>>' + " Param " + str(c) + ", quality "
                + repr(quality) + ", encoded_file: " + encoded_file
                + " size: " + str(file_size_bytes) + " bytes")
    TOTAL_BYTES[codec + metric + str(target)] += os.path.getsize(encoded_file)
    TOTAL_METRIC[codec + metric + str(target)] += quality[metric]

    if verbose > 0:
        print("[-------show-------]")
        print(
            "tuple_minus_uuid={} source_image={} width={} height={} \n"
            " codec={} c={} temp_folder={} \n metric={}  target={}  quality['ssim']={} "
            " quality['mse']={} quality['psnr']={} file_size_bytes={} encoded_file={}".format(tuple_minus_uuid,
                                                                                              source_image, width,
                                                                                              height, codec,
                                                                                              c, temp_folder,
                                                                                              metric, target,
                                                                                              quality['ssim'],
                                                                                              quality['mse'],
                                                                                              quality['psnr'],
                                                                                              file_size_bytes,
                                                                                              encoded_file))

    try:
        CONNECTION.execute(get_insert_command(), (tuple_minus_uuid, source_image, width, height, codec,
                                                  c, temp_folder, metric, target,
                                                  quality['ssim'], quality['mse'], quality['psnr'],
                                                  file_size_bytes, encoded_file))
        CONNECTION.commit()
    except:
        print("[=============]ERROR")
        CONNECTION.rollback()

    # remove_files_keeping_encode(temp_folder, encoded_file)  # comment out to keep all files


def error_function(error):
    """ error callback called when an encoding job in a worker process encounters an exception
    """
    LOGGER.error('***** ATTENTION %s', type(error))
    LOGGER.error('***** ATTENTION %s', repr(error))


def initialize_worker():
    """ method called before a worker process picks up jobs
    """
    setup_logging(worker=True, worker_id=multiprocessing.current_process().ident)
    LOGGER.info('initialize_worker() called for %s %s', multiprocessing.current_process(),
                multiprocessing.current_process().ident)


def create_table_if_needed(connection, only_perform_missing_encodes):
    if only_perform_missing_encodes:
        if connection.execute(
                ''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='ENCODES' ''').fetchall()[0][
            0] == 1:
            LOGGER.info('Will add missing entries to table')
        else:
            LOGGER.error('only_perform_missing_encodes is True but table does not exist')
            sys.exit(1)
    else:
        if connection.execute(
                ''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='ENCODES' ''').fetchall()[0][
            0] == 1:
            LOGGER.error('Table already exists. Exiting . . . ')
            sys.exit(1)
        connection.execute(get_create_table_command())


def does_entry_exist(connection, primary_key):
    res = connection.execute("SELECT * FROM ENCODES WHERE ID='{}'".format(primary_key)).fetchall()
    if len(res) > 1:
        LOGGER.error('Found more than one entry with given primary key')
        sys.exit(1)
    elif len(res) == 1:
        return True
    else:
        return False


def main(metric, target_arr, target_tol, db_file_name, only_perform_missing_encodes=False):
    """ create a pool of worker processes and submit encoding jobs, collect results and exit
    """
    setup_logging(worker=False, worker_id=multiprocessing.current_process().ident)
    LOGGER.info(
        'started main, current thread ID %s %s %s', multiprocessing.current_process(),
        multiprocessing.current_process().ident, threading.current_thread().ident)

    if only_perform_missing_encodes:
        if os.path.isfile(db_file_name):
            LOGGER.info("Will add missing entries to file " + db_file_name)
        else:
            LOGGER.error("only_perform_missing_encodes is True but db file " + db_file_name + " does not exist.")
            sys.exit(1)

    global CONNECTION
    CONNECTION = sqlite3.connect(db_file_name, check_same_thread=False)
    create_table_if_needed(CONNECTION, only_perform_missing_encodes)

    pool = multiprocessing.Pool(processes=4, initializer=initialize_worker)
    images = set(listdir_full_path('/code/images'))
    if len(images) <= 0:
        LOGGER.error(" no source files in ./images.")
        sys.exit(1)

    results = list()

    for target in target_arr:
        for num, image in enumerate(images):
            width, height, depth = get_dimensions(image)
            LOGGER.info(
                "[" + str(num) + "] Source image: " + image + " {" + width + "x" + height + "} bit-depth: " + depth)
            for codec in TUPLE_CODECS:
                LOGGER.debug(" ")
                skip_encode = False
                if only_perform_missing_encodes:
                    unique_id = make_my_tuple(image, width, height, codec.name, metric, target)
                    skip_encode = does_entry_exist(CONNECTION, unique_id)

                if not skip_encode:
                    results.append(
                        (pool.apply_async(func=bisection,
                                          args=(codec.inverse, codec.param_start, codec.param_end, codec.ab_tol,
                                                metric, target, target_tol, codec.name, image, width, height),
                                          callback=update_stats,
                                          error_callback=error_function
                                          ),
                         codec.name))
            LOGGER.info('-----------------------------------------------------------------------------------------')
    pool.close()
    pool.join()

    # time.sleep(4)
    if len(results) > 0:
        LOGGER.debug("\n\n")
        LOGGER.debug("Will get results from AsyncResult objects and list them now. This ensures callbacks complete.")
    for result in results:
        try:
            task_result = result[0].get()
            LOGGER.info(repr(task_result))
        except Exception as e:  ## all the exceptions encountered during parallel execution can be collected here at the end nicely
            LOGGER.error(repr(e))
            TOTAL_ERRORS[result[1] + result[2]] += 1

    # print("\n\n")
    # if not only_perform_missing_encodes:
    #     LOGGER.info("Total payload in kilo Bytes:")
    #     for codec in TUPLE_CODECS:
    #         for target in target_arr:
    #             if codec.name + codec.subsampling in TOTAL_ERRORS:
    #                 LOGGER.info('  {}: {} (Total errors {})'.format(codec.name + metric + str(target),
    #                                                                 TOTAL_BYTES[
    #                                                                     codec.name + metric + str(target)] / 1000.0,
    #                                                                 TOTAL_ERRORS[codec.name + metric + str(target)]))
    #             else:
    #                 LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
    #                                               TOTAL_BYTES[codec.name + metric + str(target)] / 1000.0))
    #
    #     LOGGER.info("Average metric value:")
    #     for codec in TUPLE_CODECS:
    #         for target in target_arr:
    #             if codec.name + codec.subsampling in TOTAL_ERRORS:
    #                 LOGGER.info(
    #                     '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
    #                                                         TOTAL_METRIC[codec.name + metric + str(target)]
    #                                                         / float(len(images)),
    #                                                         TOTAL_ERRORS[codec.name + metric + str(target)]))
    #             else:
    #                 LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
    #                                               TOTAL_METRIC[codec.name + metric + str(target)] / float(len(images))))
    #
    # CONNECTION.close()
    LOGGER.info("\n\n")
    LOGGER.info("[*] --------------------------- Done --------------------------- [*]")
    sys.exit(0)


if __name__ == "__main__":
    # if some encodes don't materialize, you can break out with Ctrl+C
    # then comment this out and run below for missing encodes
    main(metric='ssim', target_arr=[0.9, 0.8, 0.7], target_tol=0.005, db_file_name='encoding_results_ssim.db',
         only_perform_missing_encodes=False)
    # main(metric='ssim', target_arr=[0.92, 0.95, 0.97, 0.99], target_tol=0.005, db_file_name='encoding_results_ssim.db')
    # main(metric='psnr', target_arr=[75, 80, 85, 90, 95], target_tol=0.5, db_file_name='encoding_results_psnr.db')

    # to perform encodes targeting a certain file size
    # main(metric='file_size_bytes', target_arr=[4000, 20000, 40000, 80000], target_tol=500, db_file_name='encoding_results_file_size.db')

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
