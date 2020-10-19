import argparse
from .config_utils import boolean_string, _VERSION_

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
                        choices=['psnr_avg', 'ssim', 'vmaf', 'file_size_bytes', 'lossless'],
                        help='the target metric ')
    parser.add_argument('-db', '--db_file_name', type=str, default='encoding_results_lossless.db',
                        help="chose the db file for analyze ")
    parser.add_argument('-l', '--lossless', type=boolean_string, default=False,
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
