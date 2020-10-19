import argparse

"""
---------------------------------------------------------
1 compress in lossy model 8bits  有损8bit
---------------------------------------------------------
"""


def args_lossy_compress_config():
    parser = argparse.ArgumentParser(prog="args_lossy_compress_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
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
    parser.add_argument('-w', '--work_dir', default="/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(target_arr=get_default_target_arr(parser.parse_args().metric),
                        target_tol=get_default_target_tol(parser.parse_args().metric),
                        only_perform_missing_encodes=False,
                        db_file_name='encoding_results_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()


"""
---------------------------------------------------------
2 compress in lossless model 8bits 无损8bit
---------------------------------------------------------
"""


def args_lossless_compress_config():
    parser = argparse.ArgumentParser(prog="args_lossless_compress_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for compress result (default = 'encoding_results_lossless.db')")

    ## miner
    parser.add_argument('-np', '--num_process', type=int, default=4, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(only_perform_missing_encodes=False,
                        db_file_name='encoding_results_{}.db'.format('lossy'))
    return parser.parse_args()


"""
---------------------------------------------------------
3 compress in lossy model 16bits  有损16bit
---------------------------------------------------------
"""


def args_lossy_compress_high_dynamic_range_config():
    parser = argparse.ArgumentParser(prog="args_lossy_compress_high_dynamic_range_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-m', '--metric', type=str, default='psnr_avg',
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes'],
                        help='the metric for compare (default = psnr_avg)')
    parser.add_argument('-arr', '--target_arr', type=list, help='the target_arr (default = [20, 25, 27.5, 30, 32, 35])')

    parser.add_argument('-tol', '--target_tol', type=float, help='the target_tol (default = 0.02)')

    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    ## miner
    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for compress result (default = 'encoding_results_hdr_psnr_avg.db')")
    parser.add_argument('-np', '--num_process', default=4, type=int, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(target_arr=get_default_target_arr(parser.parse_args().metric),
                        target_tol=get_default_target_tol(parser.parse_args().metric),
                        only_perform_missing_encodes=False,
                        db_file_name='encoding_results_hdr_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()


"""
---------------------------------------------------------
4 compress in lossless model and with HDR image 无损高动态
---------------------------------------------------------
"""


def args_lossless_compress_high_dynamic_range_config():
    parser = argparse.ArgumentParser(prog="args_lossless_compress_high_dynamic_range_config",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for compress result (default = 'encoding_results_lossy.db')")

    ## miner
    parser.add_argument('-np', '--num_process', type=int, default=4, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/image_test/", type=str,
                        help='the work directory in container (default = "/image_test/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(only_perform_missing_encodes=False,
                        db_file_name='encoding_results_{}.db'.format('lossy'))
    return parser.parse_args()
