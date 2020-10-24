import multiprocessing
import os
import subprocess
import shutil
import numpy as np
from .u_ffmpeg_format import get_pixel_format
from .u_folder_build import make_dirs, check_dirs, listdir_full_path
from .u_run_cmd import run_program_no_LOGGER

"""
----------------------------------------------
CALL_FUNCTION
----------------------------------------------
"""


def error_function(error):
    """ error callback called when an encoding job in a worker process encounters an exception
    """
    print('***** ATTENTION %s', type(error))
    print('***** ATTENTION %s', repr(error))


def cp_and_rename_call(result):
    image, sequence_image = result
    print("[COPY] {} ==> {} ".format(image, sequence_image))


"""
----------------------------------------------
MAIN_FUNCTION
----------------------------------------------
"""


def copy_and_rename_file(image, target_dir, num, step):
    target_dir_aim = os.path.join(target_dir, 'YUV_{:04}'.format(int(num / step)))
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    sequence_image = os.path.join(target_dir_aim, '{}_{:04}{}'.format('IMG', num, extension))
    shutil.copyfile(image, sequence_image)
    return image, sequence_image


"""
----------------------------------------------
MAIN_SCRIPT
----------------------------------------------
"""


def COPY_AND_RENAME_SCRIPT(images, target_dir, num_process, step):
    make_dirs(target_dir)

    for num in range(int(len(images) / step) + 1):
        target_dir_aim = os.path.join(target_dir, 'YUV_{:04}'.format(num))
        check_dirs(target_dir_aim)

    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(images):
        pool.apply_async(
            func=copy_and_rename_file,
            args=(image, target_dir, num, step),
            callback=cp_and_rename_call,
            error_callback=error_function
        )
    pool.close()
    pool.join()


def PNG_TO_YUV(width, height, depth, PNG_DIR, YUV_DIR):
    images = list(set(listdir_full_path(os.path.join(PNG_DIR, os.listdir(PNG_DIR)[0]))))
    (filepath, tempfilename) = os.path.split(images[0])
    filename, extension = os.path.splitext(tempfilename)
    step = len(images)
    make_dirs(YUV_DIR)

    for num, dir in enumerate(os.listdir(PNG_DIR)):
        source_file = os.path.join(PNG_DIR, dir, "IMG_%04d{}".format(extension))
        output_fils_444 = os.path.join(YUV_DIR, "IMG_{}bit_{:04}_444.yuv".format(depth, num))
        output_fils_420 = os.path.join(YUV_DIR, "IMG_{}bit_{:04}_420.yuv".format(depth, num))
        print("[PNG_YUN] num={} dir={} -> {}".format(num, dir, output_fils_444))
        cmd = ['ffmpeg', '-f ', 'image2', '-start_number', str(step * num),
               '-i', source_file,
               '-s', "{}x{}".format(width, height),
               '-pix_fmt', get_pixel_format('444', depth), output_fils_444]
        run_program_no_LOGGER(cmd)

        print("[PNG_YUN] num={} dir={} -> {}".format(num, dir, output_fils_420))
        cmd = ['ffmpeg', '-f ', 'image2', '-start_number', str(step * num),
               '-i', source_file,
               '-s', "{}x{}".format(width, height),
               '-pix_fmt', get_pixel_format('420', depth), output_fils_420]
        run_program_no_LOGGER(cmd)


def yuv_prepare(souceimages, width, height, depth, batch_image_dir, yuv_dir, num_process, step):
    COPY_AND_RENAME_SCRIPT(souceimages, target_dir=batch_image_dir, num_process=num_process, step=step)
    PNG_TO_YUV(width, height, depth, PNG_DIR=batch_image_dir, YUV_DIR=yuv_dir)


if __name__ == '__main__':
    workdir = '/code/images/'
    batch_image_dir = os.path.join(workdir, '123')
    yuv_dir = os.path.join(workdir, '456')

    num_process = 4
    step = 12
