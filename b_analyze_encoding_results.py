import sqlite3
import numpy as np
import ntpath
import matplotlib.pyplot as plt
from collections import defaultdict

from utils.u_utils_common import easy_logging, get_mean_metric_value_file_size_bytes, get_metric_value_file_size_bytes, \
    get_print_string

from utils.u_mysql_execute import get_unique_sorted, query_for_codec, apply_checks_before_analyzing, apply_size_check, \
    get_unique_sorted_with_sub_sampling

from Config.config_analyse import args_analyze_config

blanket = 20
codec_len = 15


def eve_line(logger, all_codec_results_terse_all, source_image_all, CR):
    num_size = len(all_codec_results_terse_all)
    max_arr = np.ones(num_size) * np.inf * -1
    min_arr = np.ones(num_size) * np.inf
    for i in range(len(all_codec_results_terse_all[0])):
        consolidated_results = ""
        for k, a in enumerate(all_codec_results_terse_all):
            consolidated_results += '{:.2f}%'.format(a[i]).rjust(codec_len) if CR == False else '{:.2f}'.format(
                a[i]).rjust(codec_len)
            max_arr[k] = max(max_arr[k], a[i])
            min_arr[k] = min(min_arr[k], a[i])
        prefix = source_image_all[i]
        if CR:
            prefix += "  CR"
        if args_analyze_config().every_images == 1:
            logger.info('{}:{}'.format(str(prefix).ljust(blanket), consolidated_results))
    return (max_arr, min_arr)


def avg_line(logger, codec_results_terse, target, all_codec_results_terse_all, prefix):
    logger.info('-' * (blanket + 1 + codec_len * len(all_codec_results_terse_all)))
    consolidated_results = ""
    for a in codec_results_terse:
        consolidated_results += a.ljust(codec_len) if prefix == 'Terse' else '{:.2f}'.format(a).rjust(codec_len)
    num_size = len(all_codec_results_terse_all[0])
    t_prefix = str(
        'T {}({})'.format(target, num_size) if not args_analyze_config().lossless else 'avg {}({})'.format(prefix,
                                                                                                           num_size)).ljust(
        blanket)
    logger.info('{}:{}'.format(t_prefix, consolidated_results))


def max_min_len(logger, array_max_min, prefix):
    max_line, min_line = array_max_min
    tt = ""
    if prefix == 'Terse':
        max_line, min_line = min_line, max_line
        tt = '%'

    def tmp_line(data_line, tmp_prefix):
        consolidated_results = ""
        for a in data_line:
            consolidated_results += '{:.2f}{}'.format(a, tt).rjust(codec_len)
        prefix_ = '{} {}'.format(tmp_prefix, prefix).ljust(blanket)
        logger.info('{}:{}'.format(prefix_, consolidated_results))

    tmp_line(max_line, 'max')
    tmp_line(min_line, 'min')


def show_result(logger, codecs, sub_sampling, unique_sorted_metric_values, source_image_all, results_dict):
    print('\n')

    logger.info('=' * (blanket + 1 + codec_len * len(codecs)))
    sub_sampling_report = '{} subsampling'.format(sub_sampling)
    logger.info(sub_sampling_report)
    logger.info('-' * len(sub_sampling_report))
    codecs_string = ' ' * (blanket + 1)

    for codec in codecs:
        codecs_string += codec.rjust(codec_len)

    logger.info(codecs_string)
    for target in unique_sorted_metric_values:
        results_list_compress_rate, results_list_compress_rate_all, codec_results_terse, all_codec_results_terse_all = \
            results_dict[target]
        # ======================== EVE ======================== #
        terse_max_min = eve_line(logger, all_codec_results_terse_all, source_image_all, False)
        cr_max_min = eve_line(logger, results_list_compress_rate_all, source_image_all, True)
        # ======================== AVG ======================== #
        avg_line(logger, codec_results_terse, target, all_codec_results_terse_all, 'Terse')
        max_min_len(logger, terse_max_min, 'Terse')
        avg_line(logger, results_list_compress_rate, target, results_list_compress_rate_all, 'CR')
        max_min_len(logger, cr_max_min, 'CR')

    logger.info('=' * (blanket + 1 + codec_len * len(codecs)))
    logger.info("\n\n")


def main():
    # =================================== BASIC CONFIGS =================================== #
    metric_name = args_analyze_config().metric
    db_file_name = args_analyze_config().db_file_name
    verbose = args_analyze_config().quiet
    logger = easy_logging(file_prefix="bitrate_savings", db_file_name=db_file_name)  # logging
    connection = sqlite3.connect(db_file_name)  # connect database
    baseline_codec = args_analyze_config().baseline_codec  # baseline
    sub_sampling_arr = get_unique_sorted(connection, "SUB_SAMPLING")  # subsampling
    color_list = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray',
                  'tab:olive', 'tab:cyan']
    marker_list = ['o', 'v', '>', '<', 's', 'p', 'd', '4', 'P', 'X']  # 'D', '|', '_'
    assert len(color_list) == len(marker_list)
    # ===================================   CHECK DATA   =================================== #
    if not args_analyze_config().lossless:
        unique_sorted_metric_values, total_pixels = apply_checks_before_analyzing(connection, metric_name)
    else:
        total_pixels = apply_size_check(connection)
        unique_sorted_metric_values = get_unique_sorted(connection, "TARGET_VALUE")
    # =================================== ANALYZE  sub_sampling =================================== #
    for sub_sampling in sub_sampling_arr:
        results_dict = dict()
        results_bpp = defaultdict(list)
        results_quality = defaultdict(list)

        codecs = get_unique_sorted_with_sub_sampling(connection, "CODEC", sub_sampling)
        # ===================================     ANALYZE  target   =================================== #
        for target in unique_sorted_metric_values:

            baseline_results = connection.execute(
                query_for_codec(baseline_codec, sub_sampling, metric_name, target)).fetchall()

            baseline_metric_value, baseline_file_size, \
            baseline_count_nums, baseline_vmaf_value, \
            baseline_compress_rate = get_mean_metric_value_file_size_bytes(baseline_results)

            baseline_metric_values_all, baseline_file_size_all, \
            baselne_vmaf_values_all, baseline_source_image_all, \
            baseline_compress_rate_all, baseline_bpp_all = get_metric_value_file_size_bytes(baseline_results)

            if verbose == 0:
                print('Baseline is ' + get_print_string(baseline_codec, sub_sampling, baseline_count_nums,
                                                        baseline_metric_value,
                                                        baseline_file_size, metric_name, baseline_vmaf_value))

            results_bpp[baseline_codec].append(baseline_file_size * 8.0 / total_pixels)
            results_quality[baseline_codec].append(baseline_metric_value)

            results_list_compress_rate = list()
            results_list_compress_rate_all = list()
            results_list_terse = list()
            results_list_terse_all = list()

            # ===================================     ANALYZE  codec     =================================== #
            for codec in codecs:
                results = connection.execute(query_for_codec(codec, sub_sampling, metric_name, target)).fetchall()
                metric_value, file_size, count, vmaf_value, compress_rate = get_mean_metric_value_file_size_bytes(
                    results)
                if verbose == 0:
                    print(' Compared codec is ' + get_print_string(codec, sub_sampling, count, metric_value, file_size,
                                                                   metric_name, vmaf_value))
                    print('  Average reduction is {:.2f}%'.format(
                        (file_size - baseline_file_size) / baseline_file_size * 100.0))
                # negative is better. Positive means increase in file_size
                results_list_compress_rate.append(compress_rate)
                results_list_terse.append(
                    '{:.2f}%'.format((file_size - baseline_file_size) / baseline_file_size * 100.0).rjust(codec_len))

                metric_values_all, file_size_all, \
                vmaf_values_all, source_image_all, \
                compress_rate_all, bpp_all = get_metric_value_file_size_bytes(
                    results)

                assert source_image_all == baseline_source_image_all
                results_list_terse_all.append((np.array(file_size_all) - np.array(baseline_file_size_all)) / np.array(
                    baseline_file_size_all) * 100)
                results_list_compress_rate_all.append(compress_rate_all)

                results_bpp[codec].append(file_size * 8.0 / total_pixels)
                results_quality[codec].append(metric_value)

            results_dict[target] = (
                results_list_compress_rate, results_list_compress_rate_all, results_list_terse, results_list_terse_all)
            print("")
        # ===================================  SHOW  RESULT  =================================== #
        show_result(logger, codecs, sub_sampling, unique_sorted_metric_values, source_image_all, results_dict)

        # ===================================     PLOT     =================================== #
        fig = plt.figure(figsize=(12.8, 7.2))
        marker_num = 0
        plt.plot(results_bpp[baseline_codec], results_quality[baseline_codec], linewidth=2.0,
                 color=color_list[marker_num], marker=marker_list[marker_num], label=baseline_codec)
        for codec in codecs:
            if codec not in baseline_codec and results_quality[codec][0] < float("inf"):
                marker_num += 1
                plt.plot(results_bpp[codec], results_quality[codec], linewidth=2.0,
                         # color=color_list[marker_num],
                         # marker=marker_list[marker_num],
                         label=codec)
        plt.legend(loc='lower right')
        plt.grid()
        plt.xlabel('bit per pixel [bpp]')
        plt.ylabel(metric_name)
        plt.title('{} subsampling, using metric {}'.format(sub_sampling, metric_name.upper()))
        plt.tight_layout()
        fig.savefig('{}_{}_{}.png'.format(sub_sampling, metric_name, ntpath.basename(db_file_name)))


if __name__ == '__main__':
    main()
