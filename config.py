import argparse

_VERSION_ = 1.0


def boolean_string(s):
    if s not in {'False', 'True'}:
        raise ValueError('Not a valid boolean string')
    return s == 'True'


def get_default_target_tol(metric):
    arr = dict()
    arr['psnr_avg'] = 0.02
    arr['ssim'] = 0.002
    arr['vmaf'] = 0.2
    arr['file_size_bytes'] = 200
    return arr[metric]


def get_default_target_arr(metric):
    arr = dict()
    if metric == "file_size_bytes":
        print("[@get_default_target_arr] file_size_bytes have no default parameters")
        exit(1)
    arr['psnr_avg'] = [50]
    arr['ssim'] = [0.5, 0.8, 0.95]
    arr['vmaf'] = [50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 99]
    return arr[metric]


"""
---------------------------------------------------------
1 compress in lossy model 8bits  有损8bit
---------------------------------------------------------
"""


def args_lossy_compress_config():
    parser = argparse.ArgumentParser(prog="script_compress_parallel",
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
    parser = argparse.ArgumentParser(prog="lossy_script_compress_parallel",
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


"""
---------------------------------------------------------
3 compress in lossy model 16bits  有损16bit
---------------------------------------------------------
"""


def args_lossy_compress_high_dynamic_range_config():
    parser = argparse.ArgumentParser(prog="script_compress_parallel",
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
    parser = argparse.ArgumentParser(prog="lossy_script_compress_parallel",
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


"""
---------------------------------------------------------
4 compress in lossy model and with HDR image  有损高动态
---------------------------------------------------------
"""

"""
---------------------------------------------------------
Caculate BD-Rate
---------------------------------------------------------
"""


def args_BD_config():
    parser = argparse.ArgumentParser(prog="compute_BD_rates",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-db', '--db_file_name', type=str, default='encoding_results_psnr_avg.db',
                        help="chose the db file for caculate the BD RATES (default = 'encoding_results_psnr_avg.db')")
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    return parser.parse_args()


"""
---------------------------------------------------------
Analyze the result
---------------------------------------------------------
"""


def args_analyze_config():
    parser = argparse.ArgumentParser(prog="analyze_encoding_results",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-m', '--metric', type=str, default='psnr_avg',
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes', 'lossy'],
                        help='the target metric ')
    parser.add_argument('-db', '--db_file_name', type=str, default='encoding_results_lossy.db',
                        help="chose the db file for analyze ")
    parser.add_argument('-l', '--lossless', type=boolean_string, default=True,
                        choices=[True, False],
                        help='analyze the lossless result ?')
    ## miner
    parser.add_argument('-bc', '--baseline_codec', type=str, default='jpeg', help=" select the  baseline_codec")
    parser.add_argument('-e', '--every_images', type=int, default=1, choices=[0, 1],
                        help="to show every compress image  (default = 1)")
    parser.add_argument('-q', '--quiet', type=int, default=0, choices=[0, 1],
                        help="to show less information  (default = 0)")
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s {}'.format(_VERSION_))
    ## default
    parser.set_defaults(db_file_name='encoding_results_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()


def show_and_recode_compress_args(LOGGER, args):
    LOGGER.info("==" * 55)
    LOGGER.info("metric={}".format(args.metric))
    LOGGER.info("target_arr={}".format(args.target_arr))
    LOGGER.info("target_tol={}".format(args.target_tol))
    LOGGER.info("only_perform_missing_encodes={}".format(args.only_perform_missing_encodes))
    LOGGER.info("db_file_name={}".format(args.db_file_name))
    LOGGER.info("--" * 55)
    LOGGER.info("num_process={}".format(args.num_process))
    LOGGER.info("work_dir={}".format(args.work_dir))
    LOGGER.info("==" * 55)
    LOGGER.info("\n\n")


if __name__ == '__main__':
    # args = args_compress_config()
    #
    # print("metric={} target_tol={} target_arr={} only_perform_missing_encodes={} db_file_name={}".format(args.metric,
    #                                                                                                      args.target_tol,
    #                                                                                                      args.target_arr,
    #                                                                                                      args.only_perform_missing_encodes,
    #                                                                                                      args.db_file_name))
    args = args_lossless_compress_config()
    print("{}".format(args.num_process))
