import os
import errno
import ntpath
import glob
import logging
import statistics
import datetime
import multiprocessing



"""
----------------------------------------------
SHOW_FORMAT
----------------------------------------------
"""


def get_mean_metric_print(metric_name, metric_value, vmaf_value):
    if metric_name.upper() == 'SSIM':
        return '{:.5f} (mean VMAF {:.2f})'.format(metric_value, vmaf_value)
    else:
        return '{:.2f}'.format(metric_value)


def get_print_string(codec, sub_sampling, count, metric_value, file_size, metric_name, vmaf_value):
    line = '{} {} ({} images): mean {} {}, mean file size in bytes {}'.format(codec,
                                                                              sub_sampling,
                                                                              count,
                                                                              metric_name.upper(),
                                                                              get_mean_metric_print(metric_name,
                                                                                                    metric_value,
                                                                                                    vmaf_value),
                                                                              file_size)
    return line


"""
----------------------------------------------
METRIC
----------------------------------------------
"""


def get_metric_value_file_size_bytes(results):
    results = sorted(results, key=lambda x: x[-1])
    metric_values = [elem[0] for elem in results]
    file_size_values = [elem[1] for elem in results]
    vmaf_values = [elem[2] for elem in results]
    source_image = [elem[3] for elem in results]
    return metric_values, file_size_values, vmaf_values, source_image


def get_mean_metric_value_file_size_bytes(results):
    metric_values, file_size_values, vmaf_values, source_image = get_metric_value_file_size_bytes(results)
    return statistics.mean(metric_values), statistics.mean(file_size_values), len(metric_values), statistics.mean(
        vmaf_values)


"""
----------------------------------------------
LOGGER
----------------------------------------------
"""


def easy_logging(file_prefix, db_file_name):
    formatter = logging.Formatter('%(asctime)s - %(threadName)s - %(levelname)s - %(message)s')
    logger = logging.getLogger('report.bdrates')

    file_log_handler = logging.FileHandler(file_prefix + "_" + ntpath.basename(db_file_name) + '.txt')
    file_log_handler.setFormatter(formatter)
    logger.addHandler(file_log_handler)

    console_log_handler = logging.StreamHandler()
    console_log_handler.setFormatter(formatter)
    logger.addHandler(console_log_handler)

    logger.setLevel('DEBUG')
    return logger


"""
----------------------------------------------
DICTIONARY
----------------------------------------------
"""


def mkdir_p(path):
    """ mkdir -p
    """
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def remove_files_keeping_encode(temp_folder, encoded_file):
    """ remove files but keep encode
    """
    for f in listdir_full_path(temp_folder):
        if ntpath.basename(f) != ntpath.basename(encoded_file):
            os.remove(f)


def listdir_full_path(directory):
    """ like os.listdir(), but returns full absolute paths
    """
    for f in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, f)):
            yield os.path.abspath(os.path.join(directory, f))


def check_dirs(target_path):
    if not os.path.exists(target_path):
        os.makedirs(target_path)


def get_filename_with_temp_folder(temp_folder, filename):
    """ helper to get filename with temp folder
    """
    return os.path.join(temp_folder, filename)


def remove_exist_file(temp_folder, filename, LOGGER):
    for filePath in glob.glob(get_filename_with_temp_folder(temp_folder, filename)):
        try:
            os.remove(filePath)
        except:
            LOGGER.error("Error while deleting file : " + filePath)


"""
----------------------------------------------
OTHERS
----------------------------------------------
"""





def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value





def make_my_tuple(LOGGER, image, width, height, codec, metric, target, subsampling,param, uuid=None):
    """ make unique tuple for unique directory, primary key in DB, etc.
    """
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    my_tuple = '{filename}_{extension}_{width}x{height}_{codec}_{metric}_{target}_{subsampling}_{para}' \
        .format(filename=filename, extension=extension[1:], image=ntpath.basename(image), width=width, height=height,
                codec=codec,
                metric=metric, target=target, subsampling=subsampling,para=param)
    if uuid is not None:
        my_tuple = my_tuple + uuid
    if len(my_tuple) > 255:  # limits due to max dir name or file name length on UNIX
        LOGGER.error("ERROR : Tuple too long : " + my_tuple)
    assert len(my_tuple) < 256
    return my_tuple


def make_my_tuple_video(LOGGER, image, width, height, frames, codec, metric, target, subsampling, uuid=None):
    """ make unique tuple for unique directory, primary key in DB, etc.
    """
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    my_tuple = '{filename}_{extension}_{width}x{height}x{frames}_{codec}_{metric}_{target}_{subsampling}_' \
        .format(filename=filename, extension=extension[1:], image=ntpath.basename(image), width=width, height=height,
                frames=frames, codec=codec, metric=metric, target=target, subsampling=subsampling)
    if uuid is not None:
        my_tuple = my_tuple + uuid
    if len(my_tuple) > 255:  # limits due to max dir name or file name length on UNIX
        LOGGER.error("ERROR : Tuple too long : " + my_tuple)
    assert len(my_tuple) < 256
    return my_tuple


if __name__ == '__main__':
    pass
