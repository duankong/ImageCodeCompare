_VERSION_ = 1.0


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
