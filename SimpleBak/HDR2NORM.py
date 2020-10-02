import multiprocessing
import os
import numpy as np
from skimage import io
from skimage import exposure
import png

TARGET_IMAGE = list()


# ------------------------- utils function ------------------------------#
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


def show_images(TARGET_IMAGE):
    print("[*]------------------[*]")
    for num, image in enumerate(TARGET_IMAGE):
        print(image)
    print("[*]------------------[*]")


def check_dirs(target_path):
    if not os.path.exists(target_path):
        os.makedirs(target_path)


def get_dtpye(aim_depth):
    nptype = {8: np.uint8,
              16: np.uint16}
    return nptype[aim_depth]


def get_max_n(n):
    if n == 8:
        return 255
    if n == 16:
        return 65535
    print("[**] n=8 or 16")
    exit(1)


def listdir_full_path(directory):
    """ like os.listdir(), but returns full absolute paths
    """
    for f in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, f)):
            yield os.path.abspath(os.path.join(directory, f))


def update_image_path(result):
    image, suit_aim, width, height, depth, colorspace = result
    if suit_aim == 1:
        TARGET_IMAGE.append(image)


def save_image(n, image, enhance_Image, target_path):
    with open(get_file_path(image, target_path, "L{}".format(n)), 'wb') as f:
        writer = png.Writer(width=enhance_Image.shape[1], height=enhance_Image.shape[0], bitdepth=n,
                            greyscale=True)
        zgray2list = enhance_Image.tolist()
        writer.write(f, zgray2list)


# ------------------------- transform function ------------------------------#


def HDR_CONVERT_ENHANCE(depth, alogrithm, image, target_path, para):
    """
    enhance
    """
    data = io.imread(image)
    assert data.ndim == 2
    Source_Image = np.array(data)
    Source_Image[Source_Image > get_max_n(depth)] = get_max_n(depth)
    Source_Image = Source_Image.astype(get_dtpye(depth))
    Enhance_Image = Source_Image
    # enhance

    if alogrithm == "GAMMA":
        Enhance_Image = exposure.adjust_gamma(Source_Image, gamma=para, gain=1)
    if alogrithm == "AdaptiveHistogram":
        Enhance_Image = exposure.equalize_adapthist(Source_Image, clip_limit=0.001, nbins=get_max_n(depth))
        Enhance_Image = np.array(Enhance_Image * get_max_n(depth)).astype(get_dtpye(depth))
    # done
    save_image(depth, image, Enhance_Image, target_path)
    print("[HDR->L{}_ENHANCE] {} ... ALGORITHM={}".format(depth,
                                                          get_file_path(image, target_path,
                                                                        "L{}_enhance".format(depth)),
                                                          alogrithm))


def HDR_CONVERT_LN_with_no_transform(image, target_path, n):
    """ convert the hdr_image to rgb format n=8 ,16
    """
    data = io.imread(image)
    assert data.ndim == 2
    Ln_Image = np.array(data)
    Ln_Image[Ln_Image > get_max_n(n)] = get_max_n(n)
    Ln_Image = Ln_Image.astype(get_dtpye(n))
    save_image(n, image, Ln_Image, target_path)
    print("[HDR->L{}] {} ...".format(n, get_file_path(image, target_path, "L{}".format(n))))


# ------------------------- Script ------------------------------#

def HDR_CONVERT_ENHANCE_SCRIPT(depth, algorithm, workdir, target_path, num_process, para):
    """ convert the hdr_image (aim_depth and colorspace )to rgb format script
    """
    check_dirs(target_path=target_path)
    images = set(listdir_full_path(workdir))
    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(images):
        pool.apply_async(
            func=HDR_CONVERT_ENHANCE,
            args=(depth, algorithm, image, target_path, para),
            error_callback=error_function
        )
    pool.close()
    pool.join()
    print("\n[HDR->ENHANCE] =========== Done =========== \n")


def HDR_CONVERT_Ln_SCRIPT(n, workdir, target_path, num_process):
    """ convert the hdr_image (aim_depth and colorspace )to gray .just keep low 8 bits
    """
    check_dirs(target_path=target_path)
    images = set(listdir_full_path(workdir))
    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(images):
        pool.apply_async(
            func=HDR_CONVERT_LN_with_no_transform,
            args=(image, target_path, n),
            error_callback=error_function
        )
    pool.close()
    pool.join()
    print("\n[HDR->L{}] =========== Done =========== \n".format(n))


if __name__ == '__main__':
    # work dir
    workdir = 'F:/Desktop/deconv_xy'
    target_path_L8 = workdir + '_Ln_image'
    target_path = workdir + "_enhance_image"
    # para
    num_process = 8
    aim_depth = 16  # 8 or 16
    # script
    ALGORITHM_list = ['AdaptiveHistogram', 'GAMMA']

    HDR_CONVERT_ENHANCE_SCRIPT(depth=aim_depth, algorithm=ALGORITHM_list[0], para=0.1,
                               workdir=workdir,
                               target_path=target_path,
                               num_process=num_process)
    # HDR_CONVERT_Ln_SCRIPT(n=aim_depth, workdir=workdir, target_path=target_path_L8, num_process=num_process)
