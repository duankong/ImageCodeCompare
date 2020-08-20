import argparse


def boolean_string(s):
    if s not in {'False', 'True'}:
        raise ValueError('Not a valid boolean string')
    return s == 'True'


def get_default_target_tol(metric):
    arr = dict()
    arr['psnr_avg'] = 0.02
    arr['ssim'] = 0.005
    arr['vmaf'] = 0.2
    arr['file_size_bytes'] = 200
    return arr[metric]


def get_default_target_arr(metric):
    arr = dict()
    if metric == "file_size_bytes":
        print("[@get_default_target_arr] file_size_bytes have no default parameters")
        exit(1)
    arr['psnr_avg'] = [20, 25, 30]
    arr['ssim'] = [0.85, 0.87, 0.90, 0.92, 0.94, 0.96, 0.99]
    arr['vmaf'] = [50, 75, 85, 89, 90, 95]
    return arr[metric]


def args_config():
    parser = argparse.ArgumentParser(prog="compare_image_format",
                                     description="-------------",
                                     epilog='''the end of usage''')
    ## main
    parser.add_argument('-m', '--metric', type=str, default='psnr_avg',
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes'],
                        help='the metric for compare (default = psnr_avg)')
    parser.add_argument('-arr', '--target_arr', type=list, help='the target_arr (default = [20, 25, 27.5, 30, 32, 35])')

    parser.add_argument('-tol', '--target_tol', type=float, help='the target_tol (default = False)')

    parser.add_argument('-mis_codec', '--only_perform_missing_encodes', type=boolean_string, choices=[True, False],
                        help='only perform missing encodes (default = False)')

    parser.add_argument('-db', '--db_file_name', type=str,
                        help="the db file for ANALYZE or caculate the BD RATES (default = False)")

    ## miner
    parser.add_argument('-np', '--num_process', default=4, type=int, help='the num of process (default = 4)')
    parser.add_argument('-w', '--work_dir', default="/image_test/", type=str,
                        help='the work directory in linux (default = "/image_test/")')
    ## version
    parser.add_argument('-v', '--version', action='version', version='%(prog)s 1.0 ')
    ## default
    parser.set_defaults(target_arr=get_default_target_arr(parser.parse_args().metric),
                        target_tol=get_default_target_tol(parser.parse_args().metric),
                        only_perform_missing_encodes=False,
                        db_file_name='encoding_results_{}.db'.format(parser.parse_args().metric))
    return parser.parse_args()


def show_and_recode_args(LOGGER, args):
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
    args = args_config()

    print("metric={} target_tol={} target_arr={} only_perform_missing_encodes={} db_file_name={}".format(args.metric,
                                                                                                         args.target_tol,
                                                                                                         args.target_arr,
                                                                                                         args.only_perform_missing_encodes,
                                                                                                         args.db_file_name))
