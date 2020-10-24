import sqlite3
from statistics import mean
from collections import defaultdict

from utils.m_bd_rate_calculator import BDrateCalculator, get_rates, get_quality, get_formatted_bdrate, \
    get_formatted_mean_bdrate, my_shorten
from utils.u_mysql_execute import get_unique_sorted, get_rate_quality_points, get_unique_sources_sorted, apply_size_check
from utils.u_utils_common import easy_logging
from Config.config_analyse import args_BD_config

BD_RATE_EXCEPTION_STRING = 'BD_RATE_EXCEPTION'


def print_bd_rates(bdrates_various_metrics, codec, unique_sources, black_list_source_various_metrics,
                   list_of_metrics):
    bdrates_this_codec_various_metrics = dict()
    for metric in list_of_metrics:
        bdrates_this_codec_various_metrics[metric] = list()
    max_len_source_name = len(max(unique_sources, key=len))
    max_len_to_use_for_printing = min(80, max_len_source_name)
    for source in unique_sources:
        print_string = '  {} {}'.format(my_shorten(source,
                                                   max_len_to_use_for_printing)
                                        .ljust(max_len_to_use_for_printing),
                                        codec)
        for metric in list_of_metrics:
            print_string += ' BDRate-{} {}'.format(metric.upper(),
                                                   get_formatted_bdrate(bdrates_various_metrics[metric][codec][source]))
            if source not in black_list_source_various_metrics[metric]:
                bdrates_this_codec_various_metrics[metric].append(bdrates_various_metrics[metric][codec][source])
        print(print_string)
    result = codec.ljust(16)
    result_local = result
    for metric in list_of_metrics:
        result += '{}'.format(get_formatted_mean_bdrate(mean(bdrates_this_codec_various_metrics[metric])))
        result_local += '   Mean BDRate-{} {:.2f}'.format(metric.upper(),
                                                          mean(bdrates_this_codec_various_metrics[metric]))
    print(result_local + '\n')
    return result


def main():
    db_file_name = args_BD_config().db_file_name

    connection = sqlite3.connect(db_file_name)

    logger = easy_logging(file_prefix="bdrates", db_file_name=db_file_name)

    unique_sources = get_unique_sources_sorted(connection)
    total_pixels = apply_size_check(connection)

    baseline_codec = 'jpeg'
    sub_sampling_arr = get_unique_sorted(connection, "SUB_SAMPLING")
    codecs = get_unique_sorted(connection, "CODEC")

    metrics_for_BDRate = ['vmaf', 'ssim', 'ms_ssim', 'vif', 'psnr_y', 'psnr_avg']

    for sub_sampling in sub_sampling_arr:
        bdrates_various_metrics = dict()
        black_list_source_various_metrics = dict()
        for metric in metrics_for_BDRate:
            bdrates_various_metrics[metric] = defaultdict(dict)
            black_list_source_various_metrics[metric] = list()
        print('\n\nComputing BD rates for subsampling {}'.format(sub_sampling))

        for source in unique_sources:
            baseline_rate_quality_points = get_rate_quality_points(connection, sub_sampling, baseline_codec, source,
                                                                   total_pixels, metrics_for_BDRate)
            for codec in codecs:
                if codec == 'webp' and sub_sampling == '444':
                    continue
                rate_quality_points = get_rate_quality_points(connection, sub_sampling, codec, source, total_pixels,
                                                              metrics_for_BDRate)

                for metric in metrics_for_BDRate:
                    # print(metric.upper())
                    try:
                        bd_rate_val = 100.0 * BDrateCalculator.CalcBDRate(
                            list(zip(get_rates(baseline_rate_quality_points),
                                     get_quality(baseline_rate_quality_points, metric))),
                            list(zip(get_rates(rate_quality_points), get_quality(rate_quality_points, metric))))
                        bdrates_various_metrics[metric][codec][source] = bd_rate_val
                    except AssertionError as e:
                        print('{} {} {} {}: '.format(metric, source, codec, sub_sampling) + str(e))
                        bdrates_various_metrics[metric][codec][source] = BD_RATE_EXCEPTION_STRING
                        # BD rate computation failed for one of the codecs,
                        # so to be fair, ignore this source for final results
                        if source not in black_list_source_various_metrics[metric]:
                            black_list_source_various_metrics[metric].append(source)

        # show blacklist of the metric
        print("\n\n")
        print("{} {} {}".format("-" * 20, "BlackList_for_Metric".ljust(20), "-" * 20))
        for metric in metrics_for_BDRate:
            print('{} black list {} BD RATE\n '.format(sub_sampling.ljust(5), metric.upper().ljust(8)) + repr(
                black_list_source_various_metrics[metric]))
        results = dict()

        print("\n\n")
        print("{} {} {}".format("-" * 20, "Codecs_for_BDRate".ljust(20), "-" * 20))
        for codec in codecs:
            if codec == 'webp' and sub_sampling == '444':
                continue
            print('Codec {} subsampling {}, BD rates:'.format(codec, sub_sampling))
            result = print_bd_rates(bdrates_various_metrics, codec, unique_sources,
                                    black_list_source_various_metrics,
                                    metrics_for_BDRate)
            results[codec] = result

        print("\n\n")
        logger.info('===================' + '=' * 22 * len(metrics_for_BDRate))
        results_header = 'Results for subsampling {}'.format(sub_sampling)
        logger.info(results_header)
        logger.info('-' * len(results_header))
        table_header = 'Codec'.ljust(16)

        for metric in metrics_for_BDRate:
            table_header += ' Mean BDRate-{}'.format(metric.upper()).rjust(22)
        logger.info(table_header)

        for codec in codecs:
            if codec == 'webp' and sub_sampling == '444':
                continue
            logger.info(results[codec])
        logger.info('===================' + '=' * 22 * len(metrics_for_BDRate))


if __name__ == '__main__':
    main()
