import numpy as np
from skimage import io
import png
import os
import subprocess

from .UtilsCommon import decode


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


def show_images(TARGET_IMAGE):
    print("[*]------------------[*]")
    for num, image in enumerate(TARGET_IMAGE):
        print(image)
    print("[*]------------------[*]")


def get_dtpye(aim_depth):
    nptype = {8: np.uint8,
              16: np.uint16}
    return nptype[aim_depth]


def error_function(error):
    """ error callback called when an encoding job in a worker process encounters an exception
    """
    print('***** ATTENTION %s', type(error))
    print('***** ATTENTION %s', repr(error))


def get_file_path(image, target_path, file_type):
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    return os.path.join(target_path,
                        '{}_{}{}'.format(filename[:-3] if file_type == "HDR" else filename, file_type, extension))


def get_RG_file_path(image, target_path):
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    return os.path.join(target_path, '{}_{}{}'.format(filename, 'RG', extension))


def get_HDR_file_path(image, target_path):
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    return os.path.join(target_path, '{}_{}{}'.format(filename[:-3], 'HDR', extension))


def get_L8_file_path(image, target_path):
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    return os.path.join(target_path, '{}_{}{}'.format(filename, 'L8', extension))


def HDR_CONVERT_RG(image, target_path):
    """ convert the hdr_image to rgb format
    """
    data = io.imread(image)
    assert data.ndim == 2
    low_ = data & 0x00ff  # 取低8位 G
    high_ = data >> 8  # 取高8位 R
    last_ = np.ones(shape=data.shape, dtype=np.uint8) * 0
    RG_image = np.array([high_, low_, last_], dtype=np.uint8)
    RG_image = RG_image.transpose([1, 2, 0])
    io.imsave(os.path.join(target_path, get_file_path(image, target_path, "RG")), RG_image)
    print("[HDR->RG] {} ...".format(get_file_path(image, target_path, "RG")))


def RG_CONVERT_HDR(image, target_path, aim_depth):
    """ convert the rgb format  to hdr_image
    """
    data = io.imread(image)
    assert data.ndim == 3
    print(data.shape)
    left = np.array(data[:, :, 0]).astype(get_dtpye(aim_depth))  # 取高8位 R
    right = np.array(data[:, :, 1]).astype(get_dtpye(aim_depth))  # 取低8位 G
    left = (left << 8)
    HDR_Image = left | right
    HDR_Image_Data = np.array(HDR_Image).astype(get_dtpye(aim_depth))
    with open(get_file_path(image, target_path, "HDR"), 'wb') as f:
        writer = png.Writer(width=HDR_Image_Data.shape[0], height=HDR_Image_Data.shape[1], bitdepth=aim_depth,
                            greyscale=True)
        zgray2list = HDR_Image_Data.tolist()
        writer.write(f, zgray2list)

    # io.imsave(os.path.join(target_path, get_HDR_file_path(image, target_path)), HDR_Image_Data)
    print("[RG->HDR] {} ...".format(get_file_path(image, target_path, "HDR")))


def HDR_CONVERT_L8(image, target_path):
    """ convert the hdr_image to rgb format
    """
    data = io.imread(image)
    assert data.ndim == 2
    L8_image = np.array(data)
    L8_image[L8_image > 255] = 255
    L8_image.astype(np.uint8)
    io.imsave(os.path.join(target_path, get_file_path(image, target_path, "L8")), L8_image)
    print("[HDR->L8] {} ...".format(get_file_path(image, target_path, "L8")))


if __name__ == '__main__':
    pass
