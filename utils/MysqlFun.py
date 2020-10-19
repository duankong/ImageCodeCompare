import sys
from collections import namedtuple

RateQualityPoint = namedtuple('RateQualityPoint', ['bpp', 'quality', 'target_metric', 'target_value'])


def create_table_if_needed(LOGGER,connection, only_perform_missing_encodes):
    if only_perform_missing_encodes:
        if connection.execute(
                ''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='ENCODES' ''').fetchall()[0][
            0] == 1:
            LOGGER.info('Will add missing entries to table')
        else:
            LOGGER.error('only_perform_missing_encodes is True but table does not exist')
            exit(1)
    else:
        if connection.execute(
                ''' SELECT count(name) FROM sqlite_master WHERE type='table' AND name='ENCODES' ''').fetchall()[0][
            0] == 1:
            LOGGER.error('Table already exists. Exiting . . . ')
            exit(1)
        connection.execute(get_create_table_command())

def does_entry_exist(LOGGER, connection, primary_key):
    res = connection.execute("SELECT * FROM ENCODES WHERE ID='{}'".format(primary_key)).fetchall()
    if len(res) > 1:
        LOGGER.error('Found more than one entry with given primary key')
        exit(1)
    elif len(res) == 1:
        return True
    else:
        return False


def get_quality_dict(elem, list_of_metrics):
    quality = dict()
    for index, metric in enumerate(list_of_metrics):
        quality[metric] = elem[index]
    return quality


def get_rate_quality_points(connection, sub_sampling, codec, source, total_pixels, list_of_metrics):
    # print('{} {} {}'.format(codec, sub_sampling, source))
    csv_metrics_upper = ','.join([elem.upper() for elem in list_of_metrics])
    points = connection.execute(
        "SELECT {},FILE_SIZE_BYTES,TARGET_METRIC,TARGET_VALUE FROM ENCODES WHERE CODEC='{}' AND SUB_SAMPLING='{}' AND SOURCE='{}'"
            .format(csv_metrics_upper, codec, sub_sampling, source)).fetchall()
    rate_quality_points = [
        RateQualityPoint(elem[len(list_of_metrics)] * 8 / total_pixels, get_quality_dict(elem, list_of_metrics),
                         elem[len(list_of_metrics) + 1], elem[len(list_of_metrics) + 2]) for elem in points]
    # print(repr(rate_quality_points))
    return rate_quality_points


def query_for_codec(codec, sub_sampling, target_metric, target_value):
    return "SELECT {},FILE_SIZE_BYTES,VMAF,SOURCE FROM ENCODES WHERE CODEC='{}' AND SUB_SAMPLING='{}' AND TARGET_METRIC='{}' AND TARGET_VALUE={}" \
        .format(target_metric.upper(), codec, sub_sampling, target_metric, target_value)


def get_unique_sources_sorted(connection):
    unique_sources = connection.execute('SELECT DISTINCT SOURCE FROM ENCODES').fetchall()
    unique_sources = [elem[0] for elem in unique_sources]
    return sorted(list(set(unique_sources)))


def get_unique_sorted(connection, name):
    unique_sources = connection.execute('SELECT DISTINCT {} FROM ENCODES'.format(name)).fetchall()
    unique_sources = [elem[0] for elem in unique_sources]
    return sorted(list(set(unique_sources)))


def get_unique_sorted_with_sub_sampling(connection, name, sub_sampling):
    unique_sources = connection.execute(
        'SELECT {} FROM ENCODES WHERE SUB_SAMPLING={}'.format(name, sub_sampling)).fetchall()
    unique_sources = [elem[0] for elem in unique_sources]
    return sorted(list(set(unique_sources)))


def apply_size_check(connection):
    width_height_pairs = connection.execute('SELECT DISTINCT WIDTH,HEIGHT FROM ENCODES').fetchall()
    total_pixels = width_height_pairs[0][0] * width_height_pairs[0][1]
    for pair in width_height_pairs:
        if pair[0] * pair[1] != total_pixels:
            print('Images with different number of pixels detected in the database.')
            print('Cannot aggregate results for images with different number of pixels.')
            sys.exit(1)
    return total_pixels


def apply_checks_before_analyzing(connection, metric_name):
    # target_metrics_in_db = connection.execute('SELECT DISTINCT TARGET_METRIC FROM ENCODES').fetchall()
    target_metrics_in_db = get_unique_sorted(connection, 'TARGET_METRIC')
    if metric_name not in get_unique_sorted(connection, 'TARGET_METRIC'):
        print('Target metric {} not found in database. Target metrics in db {}.'.format(metric_name,
                                                                                        repr(target_metrics_in_db)))
        sys.exit(1)
    total_pixels = apply_size_check(connection)
    # all_metric_values = connection.execute('SELECT DISTINCT TARGET_VALUE FROM ENCODES').fetchall()
    # all_metric_values = [elem[0] for elem in all_metric_values]
    unique_sorted_metric_values = get_unique_sorted(connection, "TARGET_VALUE")
    return unique_sorted_metric_values, total_pixels


def get_insert_command():
    """ helper to get DB insert command
    """
    return '''INSERT INTO ENCODES (ID,SOURCE,WIDTH,HEIGHT,CODEC,CODEC_PARAM,
    TEMP_FOLDER,TARGET_METRIC,TARGET_VALUE,
    VMAF,SSIM,MS_SSIM,VIF,MSE_Y,MSE_U,MSE_V,MSE_AVG,PSNR_Y,PSNR_U,PSNR_V,PSNR_AVG,ADM2,
    SUB_SAMPLING,FILE_SIZE_BYTES,ENCODED_FILE) 
    VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)'''


def get_create_table_command():
    """ helper to get create table command
    """
    return '''CREATE TABLE ENCODES
         (ID TEXT PRIMARY KEY     NOT NULL,
         SOURCE           TEXT    NOT NULL,
         WIDTH            INT     NOT NULL,
         HEIGHT           INT     NOT NULL,
         CODEC            TEXT    NOT NULL,
         CODEC_PARAM      REAL    NOT NULL,
         TEMP_FOLDER      TEXT    NOT NULL,
         TARGET_METRIC    TEXT    NOT NULL,
         TARGET_VALUE     REAL    NOT NULL,
         VMAF             REAL    NOT NULL,
         SSIM             REAL    NOT NULL,
         MS_SSIM          REAL    NOT NULL,
         VIF              REAL    NOT NULL,
         MSE_Y            REAL    NOT NULL,
         MSE_U            REAL    NOT NULL,
         MSE_V            REAL    NOT NULL,
         MSE_AVG          REAL    NOT NULL,
         ADM2             REAL    NOT NULL,
         PSNR_Y           REAL    NOT NULL,
         PSNR_U           REAL    NOT NULL,
         PSNR_V           REAL    NOT NULL,
         PSNR_AVG         REAL    NOT NULL,
         SUB_SAMPLING     TEXT    NOT NULL,
         FILE_SIZE_BYTES  REAL    NOT NULL,
         ENCODED_FILE     TEXT    NOT NULL);'''


if __name__ == '__main__':
    pass
