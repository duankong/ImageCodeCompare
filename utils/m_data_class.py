import subprocess
import os
import multiprocessing
from .m_png_convert_yuv import yuv_prepare
from .u_folder_build import listdir_full_path
from .u_run_cmd import run_program_no_LOGGER


class ImageData():
    # BASIC
    image_path = []
    width = 0
    height = 0
    depth = 0
    images = []
    image_nums = 0
    # YUV
    batch_image_dir = []
    yuv_dir = []
    max_frames = 0
    yuv_dir_nums = 0

    def __init__(self, image_path):
        self.image_path = image_path
        self.width, self.height, self.depth = ImageData.get_dimensions(
            list(set(listdir_full_path(image_path)))[0])
        # self.check_data(num_process=4)
        self.images=list(set(listdir_full_path(image_path)))
        self.images.sort()
        self.image_nums = len(self.images)


    def init_yuv_info(self, batch_image_dir, yuv_dir, max_frames):
        self.batch_image_dir = batch_image_dir
        self.yuv_dir = yuv_dir
        self.max_frames = max_frames
        self.yuv_dir_nums = int(self.image_nums / self.max_frames) + 1

    def yuv_prepare(self, num_process=4):
        yuv_prepare(self.images, self.width, self.height, self.depth, self.batch_image_dir,
                    self.yuv_dir, num_process, self.max_frames)

    def check_data(self, num_process=8):
        pool = multiprocessing.Pool(processes=num_process)
        for num, image in enumerate(list(set(listdir_full_path(self.image_path)))):
            pool.apply_async(
                func=check_image_in_format,
                args=(image, self.width, self.height, self.depth),
                callback=self.update_image_path,
            )
        pool.close()
        pool.join()
        if len(self.images) <= 0:
            print("[ CHECK_DATA ] no source files in {}".format(self.image_path))
            exit(1)

    def update_image_path(self, result):
        aim_suit, image = result
        if aim_suit == 1:
            self.images.append(image)

    @staticmethod
    def split_path(path):
        (filepath, file_full_name) = os.path.split(path)
        filename, extension = os.path.splitext(file_full_name)
        return filepath, file_full_name, filename, extension

    @staticmethod
    def get_dimensions(image):
        """ given a source image, return dimensions and bit-depth
        """
        dimension_cmd = ["identify", '-format', '%w,%h,%z', image]
        res = run_program_no_LOGGER(dimension_cmd)
        res_split = res.split('\n')
        width, height, depth = res_split[len(res_split) - 1].split(
            ',')  # assuming last line has the goodies; warnings, if any, appear before
        return width, height, depth

    def print_basic_info(self):
        log = '[******************* SHOW BASIC INFO *******************]\n'
        log += 'Image_Path      : {}\n'.format(self.image_path)
        log += 'Images_nums     : {}\n'.format(self.image_nums)
        log += 'Width x Height  : {} x {}\n'.format(self.width, self.height)
        log += 'Depth           : {} bit\n'.format(self.depth)
        print(log)

    def print_yuv_info(self):
        log = '[******************* SHOW YUV INFO *******************]\n'
        log += 'Batch_Image_Dir    : {}\n'.format(self.batch_image_dir)
        log += 'YUV_Dir            : {}\n'.format(self.yuv_dir)
        log += 'YUV_Dir_nums       : {}\n'.format(self.yuv_dir_nums)
        log += 'Max_Frames         : {}\n'.format(self.max_frames)
        print(log)


def check_image_in_format(image, aim_width, aim_height, aim_depth):
    width, height, depth = ImageData.get_dimensions(image)
    aim_suit = 0
    if aim_width == width and aim_height == height and aim_depth == depth:
        aim_suit = 1
        # print('[CHECK] {}'.format(image))
    else:
        print(
            '[DATA_CHECK] ERROR : {:<20} info {:<4}x{:<4} {:<2} bit not fit {:<4}x{:<4} {:<2} bit'.format(image, width,
                                                                                                          height, depth,
                                                                                                          aim_width,
                                                                                                          aim_height,
                                                                                                          aim_depth))
    return aim_suit, image


def test():
    path = '/code/images'
    data = ImageData(path)
    data.init_yuv_info('/code/BBB/images', '/code/BBB/yuvs', 45)
    data.print_basic_info()
    data.yuv_prepare(num_process=6)
    data.print_yuv_info()


if __name__ == '__main__':
    test()
