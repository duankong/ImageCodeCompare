import sqlite3
import numpy as np
import ntpath
import matplotlib.pyplot as plt
from collections import defaultdict

from utils.UtilsCommon import easy_logging, get_mean_metric_value_file_size_bytes, get_metric_value_file_size_bytes, \
    get_print_string

from utils.MysqlFun import get_unique_sorted, query_for_codec, apply_checks_before_analyzing, apply_size_check, \
    get_unique_sorted_with_sub_sampling

from Config.config_analyse import args_analyze_config

blanket = 20
codec_len = 15


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
        all_codec_results, codec_results_terse, all_codec_results_terse_all = results_dict[target]
        # ======================== EVE ======================== #
        if args_analyze_config().every_images == 1:
            for i in range(len(all_codec_results_terse_all[0])):
                consolidated_results = ""
                for a in all_codec_results_terse_all:
                    consolidated_results += '{:.2f}%'.format(a[i]).rjust(codec_len)
                logger.info('{}:{}'.format(str(source_image_all[i]).ljust(blanket), consolidated_results))
        # ======================== AVG ======================== #
        consolidated_results = ""
        for a in codec_results_terse:
            consolidated_results += a.ljust(codec_len)
        logger.info('{}:{}'.format(str('T {}({})'.format(target, len(
            all_codec_results_terse_all[0])) if not args_analyze_config().lossless else 'avg({})'.format(
            len(all_codec_results_terse_all[0]))).ljust(blanket),
                                   consolidated_results))
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

            baseline_metric_value, baseline_file_size, baseline_count, baseline_vmaf_value = get_mean_metric_value_file_size_bytes(
                baseline_results)

            baseline_metric_values_all, baseline_file_size_all, baselne_vmaf_values_all, baseline_source_image_all = get_metric_value_file_size_bytes(
                baseline_results)
            if verbose == 0:
                print('Baseline is ' + get_print_string(baseline_codec, sub_sampling, baseline_count,
                                                        baseline_metric_value,
                                                        baseline_file_size, metric_name, baseline_vmaf_value))

            results_bpp[baseline_codec].append(baseline_file_size * 8.0 / total_pixels)
            results_quality[baseline_codec].append(baseline_metric_value)

            results_list = list()
            results_list_terse = list()
            results_list_terse_all = list()
            # ===================================     ANALYZE  codec     =================================== #
            for codec in codecs:
                results = connection.execute(query_for_codec(codec, sub_sampling, metric_name, target)).fetchall()
                metric_value, file_size, count, vmaf_value = get_mean_metric_value_file_size_bytes(results)
                if verbose == 0:
                    print(' Compared codec is ' + get_print_string(codec, sub_sampling, count, metric_value, file_size,
                                                                   metric_name, vmaf_value))
                    print('  Average reduction is {:.2f}%'.format(
                        (file_size - baseline_file_size) / baseline_file_size * 100.0))
                # negative is better. Positive means increase in file_size
                results_list.append(
                    '{} {:.2f}%'.format(codec, (file_size - baseline_file_size) / baseline_file_size * 100.0))
                results_list_terse.append(
                    '{:.2f}%'.format((file_size - baseline_file_size) / baseline_file_size * 100.0).rjust(codec_len))

                metric_values_all, file_size_all, vmaf_values_all, source_image_all = get_metric_value_file_size_bytes(
                    results)

                assert source_image_all == baseline_source_image_all
                results_list_terse_all.append((np.array(file_size_all) - np.array(baseline_file_size_all)) / np.array(
                    baseline_file_size_all) * 100)

                results_bpp[codec].append(file_size * 8.0 / total_pixels)
                results_quality[codec].append(metric_value)

            results_dict[target] = (results_list, results_list_terse, results_list_terse_all)
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
