import multiprocessing
from utils.UtilsCommon import listdir_full_path, check_dirs
from utils.Test5Common import show_images, error_function, HDR_CONVERT_RG, RG_CONVERT_HDR, HDR_CONVERT_L8, \
    get_dimensions

TARGET_IMAGE = list()


def update_image_path(result):
    image, suit_aim, width, height, depth, colorspace = result
    if suit_aim == 1:
        TARGET_IMAGE.append(image)


def get_high_range_depth_image(workdir, num_process, aim_depth, aim_colorspace):
    TARGET_IMAGE.clear()
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
    if len(TARGET_IMAGE) <= 0:
        print("[ERROR] no source files in {}".format(workdir))
        exit(1)


"""
----------------------------------------------
SCRIPT
----------------------------------------------
"""


def HDR_CONVERT_L8_SCRIPT(workdir, target_path, num_process, aim_depth, colorspace='Gray'):
    """ convert the hdr_image (aim_depth and colorspace )to gray .just keep low 8 bits
    """
    check_dirs(target_path=target_path)
    get_high_range_depth_image(workdir, num_process, aim_depth, colorspace)

    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(TARGET_IMAGE):
        pool.apply_async(
            func=HDR_CONVERT_L8,
            args=(image, target_path),
            error_callback=error_function
        )
    pool.close()
    pool.join()
    print("\n[HDR->L8] =========== Done =========== \n")


def HDR_CONVERT_RG_SCRIPT(workdir, target_path, num_process, aim_depth, colorspace='Gray'):
    """ convert the hdr_image (aim_depth and colorspace )to rgb format script
    """
    check_dirs(target_path=target_path)
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
    print("\n[HDR->RG] =========== Done =========== \n")


def RG_CONVERT_HDR_SCRIPT(workdir, target_path, num_process, aim_depth):
    """ convert the rgb format  to hdr_image script
    """
    check_dirs(target_path=target_path)
    images = set(listdir_full_path(workdir))
    if len(images) <= 0:
        print(" no source files in {}".format(workdir))
        exit(1)
    pool = multiprocessing.Pool(processes=num_process)
    for num, image in enumerate(images):
        pool.apply_async(
            func=RG_CONVERT_HDR,
            args=(image, target_path, aim_depth),
            error_callback=error_function
        )
    pool.close()
    pool.join()
    print("\n[RG->HDR] =========== Done =========== \n")


if __name__ == '__main__':
    workdir = '/code/images/'
    target_path = workdir + 'RG_image'
    target_path_L8 = workdir + 'L8_image'
    back_path = workdir + 'BK_image'

    num_process = 8
    aim_depth = 16
    # convert RG
    HDR_CONVERT_RG_SCRIPT(workdir=workdir, target_path=target_path, num_process=num_process, aim_depth=aim_depth)
    RG_CONVERT_HDR_SCRIPT(workdir=target_path, target_path=back_path, num_process=num_process, aim_depth=aim_depth)
    # # keep low 8 bit
    HDR_CONVERT_L8_SCRIPT(workdir=workdir, target_path=target_path_L8, num_process=num_process, aim_depth=aim_depth)
