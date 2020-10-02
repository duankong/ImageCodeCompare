import multiprocessing
import os
import subprocess
import shutil
import numpy as np

TARGET_IMAGE = list()
DEPTH = None
"""
----------------------------------------------
OTHERS
----------------------------------------------
"""


def make_dirs(target_path):
    if os.path.exists(target_path):
        shutil.rmtree(target_path)
    os.makedirs(target_path)


def check_dirs(target_path):
    if not os.path.exists(target_path):
        os.makedirs(target_path)


def listdir_full_path(directory):
    """ like os.listdir(), but returns full absolute paths
    """
    for f in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, f)):
            yield os.path.abspath(os.path.join(directory, f))


"""
----------------------------------------------
PIXEL_FORMAT
----------------------------------------------
"""


def get_pixel_format(subsampling, depth):
    """ helper to set pixel format from subsampling, assuming full range,
        for converting source to yuv prior to encoding
    """
    pixel_format = None
    if depth == '8':
        if subsampling == '420':
            pixel_format = 'yuvj420p'
        elif subsampling == '444':
            pixel_format = 'yuvj444p'
        else:
            raise RuntimeError('Unsupported subsampling ' + subsampling)
    elif depth == '16':
        if subsampling == '420':
            pixel_format = 'yuv420p16le'
        elif subsampling == '444':
            pixel_format = 'yuv444p16le'
        else:
            raise RuntimeError('Unsupported subsampling ' + subsampling)
    else:
        raise RuntimeError('Unsupported depth ' + depth)
    return pixel_format


"""
----------------------------------------------
RUN_CMD
----------------------------------------------
"""


def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


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
        print("***** ATTENTION : subprocess call crashed: %s\n%s", args, e.output)
        raise


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


def update_image_path(result):
    aim_suit, image = result
    if aim_suit == 1:
        TARGET_IMAGE.append(image)


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


def get_dimensions(image):
    """ given a source image, return dimensions and bit-depth
    """
    dimension_cmd = ["identify", '-format', '%w,%h,%z', image]
    res = run_program(dimension_cmd)
    res_split = res.split('\n')
    width, height, depth = res_split[len(res_split) - 1].split(
        ',')  # assuming last line has the goodies; warnings, if any, appear before
    return width, height, depth


def check_image_in_format(image, aim_width, aim_height, aim_depth):
    width, height, depth = get_dimensions(image)
    aim_suit = 1 if aim_width == width and aim_height == height and aim_depth == depth else 0
    return aim_suit, image


"""
----------------------------------------------
MAIN_SCRIPT
----------------------------------------------
"""


def GET_PNG_SEQUENCE_SCRIPT(workdir, num_process):
    images = list(set(listdir_full_path(workdir)))
    width, height, depth = get_dimensions(images[0])

    print("[CHECK] width={} height={} depth={}".format(width, height, depth))
    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(images):
        pool.apply_async(
            func=check_image_in_format,
            args=(image, width, height, depth),
            callback=update_image_path,
            error_callback=error_function
        )
    pool.close()
    pool.join()
    if len(TARGET_IMAGE) <= 0:
        print("[ERROR] no source files in {}".format(workdir))
        exit(1)
    TARGET_IMAGE.sort()


def COPY_AND_RENAME_SCRIPT(target_dir, num_process, step):
    make_dirs(target_dir)

    for num in range(int(len(TARGET_IMAGE) / step) + 1):
        target_dir_aim = os.path.join(target_dir, 'YUV_{:04}'.format(num))
        check_dirs(target_dir_aim)

    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(TARGET_IMAGE):
        pool.apply_async(
            func=copy_and_rename_file,
            args=(image, target_dir, num, step),
            callback=cp_and_rename_call,
            error_callback=error_function
        )
    pool.close()
    pool.join()


def PNG_TO_YUV(PNG_DIR, YUV_DIR):
    images = list(set(listdir_full_path(os.path.join(PNG_DIR, os.listdir(PNG_DIR)[0]))))
    width, height, depth = get_dimensions(images[0])
    (filepath, tempfilename) = os.path.split(images[0])
    filename, extension = os.path.splitext(tempfilename)
    step = len(images)

    for num, dir in enumerate(os.listdir(PNG_DIR)):
        source_file = os.path.join(PNG_DIR, dir, "IMG_%04d{}".format(extension))
        output_fils_444 = os.path.join(YUV_DIR, "IMG_{}bit_{:04}_444.yuv".format(depth, num))
        output_fils_420 = os.path.join(YUV_DIR, "IMG_{}bit_{:04}_420.yuv".format(depth, num))
        print("[PNG_YUN] num={} dir={} -> {}".format(num, dir, output_fils_444))
        cmd = ['ffmpeg', '-f ', 'image2', '-start_number', str(step * num),
               ' -i', source_file,
               '-s', "{}x{}".format(width, height),
               '-pix_fmt', get_pixel_format('444', depth), output_fils_444]
        run_program(cmd)

        print("[PNG_YUN] num={} dir={} -> {}".format(num, dir, output_fils_420))
        cmd = ['ffmpeg', '-f ', 'image2', '-start_number', str(step * num),
               ' -i', source_file,
               '-s', "{}x{}".format(width, height),
               '-pix_fmt', get_pixel_format('420', depth), output_fils_420]
        run_program(cmd)



def main(workdir, batch_image_dir, num_process, step):
    GET_PNG_SEQUENCE_SCRIPT(workdir=workdir, num_process=num_process)
    COPY_AND_RENAME_SCRIPT(target_dir=batch_image_dir, num_process=num_process, step=step)
    PNG_TO_YUV(PNG_DIR=batch_image_dir, YUV_DIR=batch_image_dir)


if __name__ == '__main__':
    workdir = '/code/images/'
    batch_image_dir = workdir + 'YUV_SOURCE'

    num_process = 8
    step = 12

    main(workdir=workdir, batch_image_dir=batch_image_dir, num_process=num_process, step=step)
