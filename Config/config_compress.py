import argparse

from .config_utils import boolean_string, _VERSION_


def get_default_target_tol(metric):
    arr = dict()
    arr['psnr_avg'] = 1
    arr['ssim'] = 0.002
    arr['vmaf'] = 0.2
    arr['file_size_bytes'] = 200
    return arr[metric]


def get_default_target_arr(metric):
    arr = dict()
    if metric == "file_size_bytes":
        print("[@get_default_target_arr] file_size_bytes have no default parameters")
        exit(1)
    arr['psnr_avg'] = [37]
    arr['ssim'] = [0.5, 0.8, 0.95, 0.99]
    arr['vmaf'] = [50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 99]
    return arr[metric]


"""
---------------------------------------------------------
00_00 compress in lossy/lossless model 8bits  有损无损8bit
---------------------------------------------------------
"""


def args_8bit_image_compress_config():
    parser = argparse.ArgumentParser(prog="args_lossy_compress_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-l', '--lossless', type=boolean_string, choices=[True, False],
                        help='execute lossless code (default=False)')
    parser.add_argument('-m', '--metric', type=str, default='psnr_avg',
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes'],
                        help='the metric for compare (default = psnr_avg)')
    parser.add_argument('-arr', '--target_arr', type=list, help='the target_arr (default = [20, 25, 27.5, 30, 32, 35])')

    parser.add_argument('-tol', '--target_tol', type=float, help='the target_tol (default = 0.02)')

    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    ## miner
    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for compress result (default = 'encoding_results_psnr_avg.db')")
    parser.add_argument('-np', '--num_process', default=4, type=int, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/code/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')
    parser.add_argument('-im', '--image_path', default="/code/images/", type=str,
                        help='the work directory in container (default = "/code/images/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(target_arr=get_default_target_arr(parser.parse_args().metric),
                        target_tol=get_default_target_tol(parser.parse_args().metric),
                        only_perform_missing_encodes=False,
                        lossless=False,
                        db_file_name='encoding_results_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()


"""
---------------------------------------------------------
00_11 compress in lossy/lossless model 16bits  有损无损16bit
---------------------------------------------------------
"""


def args_16bit_image_compress_config():
    parser = argparse.ArgumentParser(prog="args_lossy_compress_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-l', '--lossless', type=boolean_string, choices=[True, False],
                        help='execute lossless code (default=False)')
    parser.add_argument('-m', '--metric', type=str, default='psnr_avg',
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes'],
                        help='the metric for compare (default = psnr_avg)')
    parser.add_argument('-arr', '--target_arr', type=list, help='the target_arr (default = [20, 25, 27.5, 30, 32, 35])')

    parser.add_argument('-tol', '--target_tol', type=float, help='the target_tol (default = 0.02)')

    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    ## miner
    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for compress result (default = 'encoding_results_psnr_avg.db')")
    parser.add_argument('-np', '--num_process', default=4, type=int, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/code/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')
    parser.add_argument('-im', '--image_path', default="/code/images/", type=str,
                        help='the work directory in container (default = "/code/images/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(target_arr=get_default_target_arr(parser.parse_args().metric),
                        target_tol=get_default_target_tol(parser.parse_args().metric),
                        only_perform_missing_encodes=False,
                        lossless=True,
                        db_file_name='encoding_results_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()


"""
---------------------------------------------------------
5 compress in video 视频压缩 有损 无损 8bit
---------------------------------------------------------
"""


def args_compress_video_config():
    parser = argparse.ArgumentParser(prog="args_compress_video_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-i', '--image_path', type=str, default='/code/images',
                        help='souce image path (default =  /code/images)')
    parser.add_argument('-bid', '--batch_image_dir', type=str, default='/code/images/yuv_source',
                        help='batch image path (default =  /code/images/yuv_source)')
    parser.add_argument('-yd', '--yuv_dir', type=str, default='/code/images/yuv_source',
                        help='yuv path (default =  /code/images/yuv_source)')
    parser.add_argument('-f', '--yuv_frames', type=int, help='every yuv frames(default=30)')

    parser.add_argument('-m', '--metric', type=str, default='psnr_avg',
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes', 'lossless'],
                        help='the target metric ')
    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for compress result (default = 'encoding_results_lossy.db')")
    parser.add_argument('-yuv', '--prepare_yuv', type=boolean_string, choices=[True, False],
                        help='execute convert images to yuv (default=False)')
    parser.add_argument('-l', '--lossless', type=boolean_string, choices=[True, False],
                        help='execute lossless code (default=False)')
    ## miner
    parser.add_argument('-arr', '--target_arr', type=list, help='the target_arr (default = [20, 25, 27.5, 30, 32, 35])')

    parser.add_argument('-tol', '--target_tol', type=float, help='the target_tol (default = 0.02)')

    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    parser.add_argument('-np', '--num_process', type=int, default=4, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/code/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')

    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(target_arr=get_default_target_arr(parser.parse_args().metric),
                        target_tol=get_default_target_tol(parser.parse_args().metric),
                        only_perform_missing_encodes=False,
                        prepare_yuv=False,
                        lossless=True,
                        batch_image_dir='/code/CCC/batch',
                        yuv_dir='/code/CCC/yuvs',
                        yuv_frames=8,
                        db_file_name='encoding_results_video_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()




if __name__ == '__main__':
    pass
