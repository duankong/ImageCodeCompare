import multiprocessing
import subprocess
import os
import time
import numpy as np
from skimage import io

TARGET_IMAGE = list()


def listdir_full_path(directory):
    """ like os.listdir(), but returns full absolute paths
    """
    for f in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, f)):
            yield os.path.abspath(os.path.join(directory, f))


def error_function(error):
    """ error callback called when an encoding job in a worker process encounters an exception
    """
    print('***** ATTENTION %s', type(error))
    print('***** ATTENTION %s', repr(error))


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


def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def get_dimensions(image, aim_depth, aim_colorspace):
    """ given a source image, return dimensions and bit-depth
    """
    aim_depth = str(aim_depth)
    dimension_cmd = ["identify", '-format', '%w,%h,%z,%[colorspace]', image]
    res = run_program(dimension_cmd)
    res_split = res.split('\n')
    width, height, depth, colorspace = res_split[len(res_split) - 1].split(',')
    suit_aim = 1 if (depth == aim_depth and colorspace == aim_colorspace) else 0
    # print("find={} || image={} || depth={} || suit_aim={} || colorspace={}".format(suit_aim, image, depth, suit_aim,
    #                                                                                colorspace))
    return image, suit_aim, width, height, depth, colorspace


def update_image_path(result):
    image, suit_aim, width, height, depth, colorspace = result
    if suit_aim == 1:
        TARGET_IMAGE.append(image)


def get_high_range_depth_image(workdir, num_process, aim_depth, aim_colorspace):
    pool = multiprocessing.Pool(processes=num_process)
    images = set(listdir_full_path(workdir))
    for num, image in enumerate(images):
        pool.apply_async(
            func=get_dimensions,
            args=(image, aim_depth, aim_colorspace),
            callback=update_image_path,
            error_callback=error_function
        )
    pool.close()
    pool.join()


def get_RG_file_path(image, target_path):
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    return os.path.join(target_path, '{}_{}{}'.format(filename, 'RG', extension))


def HDR_CONVERT_RG(image, target_path):
    data = io.imread(image)
    assert data.ndim == 2
    right = data & 0x00ff  # 取右bai8位
    left = data >> 8  # 取左8位
    last = np.zeros(shape=data.shape, dtype=np.uint8)
    print(right.dtype)
    io.imsave(os.path.join(target_path, get_RG_file_path(image, target_path)), right)


def main(workdir, target_path, num_process, aim_depth, colorspace):
    get_high_range_depth_image(workdir, num_process, aim_depth, colorspace)
    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(TARGET_IMAGE):
        pool.apply_async(
            func=HDR_CONVERT_RG,
            args=(image, target_path),
            error_callback=error_function
        )
    pool.close()
    pool.join()


def show_images():
    print("[*]------------------[*]")
    for num, image in enumerate(TARGET_IMAGE):
        print(image)
    print("[*]------------------[*]")


if __name__ == '__main__':
    workdir = '/code/images'
    target_path = '/code/images/target_path'
    aim_depth = 8
    colorspace = 'Gray'  # Gray sRGB
    main(workdir=workdir, target_path=target_path, num_process=4, aim_depth=aim_depth, colorspace=colorspace)
    # show_images()
