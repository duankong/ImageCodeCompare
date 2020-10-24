#  8 bit and 16 bit image config
from Config.image_8bit_config import tuple_codes, lossless_tuple_codes
from Config.image_16bit_config import tuple_codes_high_dynamic_range, lossless_tuple_codes_high_dynamic_range

from Config.video_8bit_config import video_tuple_lossless_codes, video_tuple_codes

_VERSION_ = 1.0


def video_tuple_choice(LOGGER, depth, func_choice):
    TUPLE_CODECS = None
    if depth == '8':
        if func_choice == 'lossless':
            TUPLE_CODECS = video_tuple_lossless_codes()
        elif func_choice in ['customize', 'auto']:
            TUPLE_CODECS = video_tuple_codes()
        else:
            LOGGER.error("[tuple_choice] Not support tuple in func_choice={}".format(func_choice))
    else:
        LOGGER.error("[tuple_choice] Not support tuple in depth = {}".format(depth))
    return TUPLE_CODECS


def image_tuple_choice(LOGGER, depth, func_choice):
    TUPLE_CODECS = None
    if depth == '8':
        if func_choice == 'lossless':
            TUPLE_CODECS = lossless_tuple_codes()
        elif func_choice in ['customize', 'auto']:
            TUPLE_CODECS = tuple_codes()
        else:
            LOGGER.error("[tuple_choice] Not support tuple in func_choice={}".format(func_choice))
    elif depth == '16':
        if func_choice == 'lossless':
            TUPLE_CODECS = lossless_tuple_codes_high_dynamic_range()
        elif func_choice in ['customize', 'auto']:
            TUPLE_CODECS = tuple_codes_high_dynamic_range()
        else:
            LOGGER.error("[tuple_choice] Not support tuple in func_choice = {}".format(func_choice))
    else:
        LOGGER.error("[tuple_choice] Not support tuple in depth = {}".format(depth))
    return TUPLE_CODECS


def boolean_string(s):
    if s not in {'False', 'True'}:
        raise ValueError('Not a valid boolean string')
    return s == 'True'


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
    pass
