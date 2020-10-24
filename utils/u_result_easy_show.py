def show_image_lossy_result(results, only_perform_missing_encodes, LOGGER, TOTAL_ERRORS, TOTAL_METRIC, TOTAL_BYTES,
                            TUPLE_CODECS, data, target_arr, metric):
    LOGGER.debug('\n\n')
    LOGGER.debug("Will get results from AsyncResult objects and list them now. This ensures callbacks complete.")
    for result in results:
        try:
            task_result = result[0].get()
            LOGGER.info(repr(task_result))
        except Exception as e:  ## all the exceptions encountered during parallel execution can be collected here at the end nicely
            LOGGER.error(repr(e))
            TOTAL_ERRORS[result[1] + result[2]] += 1

    LOGGER.info("\n\n")
    if not only_perform_missing_encodes:
        LOGGER.info("Total payload in kilo Bytes:")
        for codec in TUPLE_CODECS:
            for target in target_arr:
                if codec.name + codec.subsampling in TOTAL_ERRORS:
                    LOGGER.info(
                        '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                            TOTAL_BYTES[codec.name + codec.subsampling + metric + str(
                                                                target)] / 1000.0,
                                                            TOTAL_ERRORS[
                                                                codec.name + codec.subsampling + metric + str(target)]))
                else:
                    LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                                  TOTAL_BYTES[
                                                      codec.name + codec.subsampling + metric + str(target)] / 1000.0))

        LOGGER.info("Average metric value:")
        for codec in TUPLE_CODECS:
            for target in target_arr:
                if codec.name + codec.subsampling in TOTAL_ERRORS:
                    LOGGER.info(
                        '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                            TOTAL_METRIC[
                                                                codec.name + codec.subsampling + metric + str(target)]
                                                            / float(data.image_nums),
                                                            TOTAL_ERRORS[
                                                                codec.name + codec.subsampling + metric + str(target)]))
                else:
                    LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                                  TOTAL_METRIC[
                                                      codec.name + codec.subsampling + metric + str(target)] / float(
                                                      data.image_nums)))


def show_image_lossless_result(results, only_perform_missing_encodes, LOGGER, TOTAL_ERRORS, TOTAL_METRIC, TOTAL_BYTES,
                               TUPLE_CODECS, target, metric, images_nums):
    LOGGER.debug('\n\n')
    LOGGER.debug("Will get results from AsyncResult objects and list them now. This ensures callbacks complete.")
    for result in results:
        try:
            task_result = result[0].get()
            LOGGER.info(repr(task_result))
        except Exception as e:  ## all the exceptions encountered during parallel execution can be collected here at the end nicely
            LOGGER.error(repr(e))
            TOTAL_ERRORS[result[1] + result[2]] += 1

    LOGGER.info("\n\n")
    if not only_perform_missing_encodes:
        LOGGER.info("Total payload in kilo Bytes:")
        for codec in TUPLE_CODECS:
            if codec.name + codec.subsampling in TOTAL_ERRORS:
                LOGGER.info(
                    '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                        TOTAL_BYTES[codec.name + codec.subsampling + metric + str(
                                                            target)] / 1000.0,
                                                        TOTAL_ERRORS[
                                                            codec.name + codec.subsampling + metric + str(target)]))
            else:
                LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                              TOTAL_BYTES[
                                                  codec.name + codec.subsampling + metric + str(target)] / 1000.0))

        LOGGER.info("Average metric value:")
        for codec in TUPLE_CODECS:
            if codec.name + codec.subsampling in TOTAL_ERRORS:
                LOGGER.info(
                    '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                        TOTAL_METRIC[
                                                            codec.name + codec.subsampling + metric + str(target)]
                                                        / float(images_nums),
                                                        TOTAL_ERRORS[
                                                            codec.name + codec.subsampling + metric + str(target)]))
            else:
                LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                              TOTAL_METRIC[
                                                  codec.name + codec.subsampling + metric + str(target)] / float(
                                                  images_nums)))


def result_video_show(LOGGER, images, results, TOTAL_ERRORS, TUPLE_CODECS, TOTAL_METRIC,
                      TOTAL_BYTES, only_perform_missing_encodes, frames, metric, target_arr):
    LOGGER.debug('\n\n')
    LOGGER.debug("Will get results from AsyncResult objects and list them now. This ensures callbacks complete.")
    for result in results:
        try:
            task_result = result[0].get()
            LOGGER.info(repr(task_result))
        except Exception as e:  ## all the exceptions encountered during parallel execution can be collected here at the end nicely
            LOGGER.error(repr(e))
            TOTAL_ERRORS[result[1] + result[2]] += 1

    LOGGER.info("\n\n")
    if not only_perform_missing_encodes:
        LOGGER.info("Total payload in kilo Bytes:")
        for codec in TUPLE_CODECS:
            for target in target_arr:
                if codec.name + codec.subsampling in TOTAL_ERRORS:
                    LOGGER.info(
                        '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                            TOTAL_BYTES[codec.name + codec.subsampling + metric + str(
                                                                target)] / 1000.0,
                                                            TOTAL_ERRORS[
                                                                codec.name + codec.subsampling + metric + str(target)]))
                else:
                    LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                                  TOTAL_BYTES[
                                                      codec.name + codec.subsampling + metric + str(target)] / 1000.0))

        LOGGER.info("Average metric value:")
        for codec in TUPLE_CODECS:
            for target in target_arr:
                if codec.name + codec.subsampling in TOTAL_ERRORS:
                    LOGGER.info(
                        '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                            TOTAL_METRIC[
                                                                codec.name + codec.subsampling + metric + str(target)]
                                                            / float(len(images)),
                                                            TOTAL_ERRORS[
                                                                codec.name + codec.subsampling + metric + str(target)]))
                else:
                    LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                                  TOTAL_METRIC[
                                                      codec.name + codec.subsampling + metric + str(target)] / float(
                                                      1)))
                    # int(len(images) / frames) + 1)))


def result_lossless_show(LOGGER, images, results, TOTAL_ERRORS, TUPLE_CODECS, TOTAL_METRIC,
                         TOTAL_BYTES, only_perform_missing_encodes, frames, metric, target):
    LOGGER.debug('\n\n')
    LOGGER.debug("Will get results from AsyncResult objects and list them now. This ensures callbacks complete.")
    for result in results:
        try:
            task_result = result[0].get()
            LOGGER.info(repr(task_result))
        except Exception as e:  ## all the exceptions encountered during parallel execution can be collected here at the end nicely
            LOGGER.error(repr(e))
            TOTAL_ERRORS[result[1] + result[2]] += 1

    LOGGER.info("\n\n")
    if not only_perform_missing_encodes:
        LOGGER.info("Total payload in kilo Bytes:")
        for codec in TUPLE_CODECS:
            if codec.name + codec.subsampling in TOTAL_ERRORS:
                LOGGER.info(
                    '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                        TOTAL_BYTES[codec.name + codec.subsampling + metric + str(
                                                            target)] / 1000.0,
                                                        TOTAL_ERRORS[
                                                            codec.name + codec.subsampling + metric + str(target)]))
            else:
                LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                              TOTAL_BYTES[
                                                  codec.name + codec.subsampling + metric + str(target)] / 1000.0))

        LOGGER.info("Average metric value:")
        for codec in TUPLE_CODECS:
            if codec.name + codec.subsampling in TOTAL_ERRORS:
                LOGGER.info(
                    '  {}: {} (Total errors {})'.format(codec.name + codec.subsampling + metric + str(target),
                                                        TOTAL_METRIC[
                                                            codec.name + codec.subsampling + metric + str(target)]
                                                        / float(len(images)),
                                                        TOTAL_ERRORS[
                                                            codec.name + codec.subsampling + metric + str(target)]))
            else:
                LOGGER.info('  {}: {}'.format(codec.name + codec.subsampling + metric + str(target),
                                              TOTAL_METRIC[
                                                  codec.name + codec.subsampling + metric + str(target)] / float(
                                                  1.0)))  # int(len(images) / frames) + 1


if __name__ == '__main__':
    pass
