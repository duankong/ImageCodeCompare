import os
import errno
import ntpath
import glob
import logging
import statistics
import datetime


def get_metric_value_file_size_bytes(results):
    metric_values = [elem[0] for elem in results]
    file_size_values = [elem[1] for elem in results]
    vmaf_values = [elem[2] for elem in results]
    source_image = [elem[3] for elem in results]
    return metric_values, file_size_values, vmaf_values, source_image


def get_mean_metric_value_file_size_bytes(results):
    metric_values, file_size_values, vmaf_values, source_image = get_metric_value_file_size_bytes(results)
    return statistics.mean(metric_values), statistics.mean(file_size_values), len(metric_values), statistics.mean(
        vmaf_values)


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


def setup_logging(LOGGER, worker, worker_id):
    """
    set up logging for the process calling this function.
    :param worker: True means it is a worker process from the pool. False means it is the main process.
    :param worker_id: unique ID identifying the process
    :return:
    """
    for h in list(LOGGER.handlers):
        LOGGER.removeHandler(h)
    now = datetime.datetime.now()
    formatter = logging.Formatter('%(asctime)s - %(threadName)s - %(levelname)s - %(message)s')
    name = "compression_results_"
    if worker:
        name += "worker_"
        global CONNECTION
        CONNECTION = None
    name += str(worker_id) + '_' + now.isoformat() + '.txt'
    file_log_handler = logging.FileHandler(name)
    file_log_handler.setFormatter(formatter)
    LOGGER.addHandler(file_log_handler)

    console_log_handler = logging.StreamHandler()
    console_log_handler.setFormatter(formatter)
    LOGGER.addHandler(console_log_handler)

    LOGGER.setLevel('DEBUG')


def make_my_tuple(LOGGER, image, width, height, codec, metric, target, subsampling, uuid=None):
    """ make unique tuple for unique directory, primary key in DB, etc.
    """
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    my_tuple = '{filename}_{extension}_{width}x{height}_{codec}_{metric}_{target}_{subsampling}_' \
        .format(filename=filename, extension=extension[1:], image=ntpath.basename(image), width=width, height=height,
                codec=codec,
                metric=metric, target=target, subsampling=subsampling)
    if uuid is not None:
        my_tuple = my_tuple + uuid
    if len(my_tuple) > 255:  # limits due to max dir name or file name length on UNIX
        LOGGER.error("ERROR : Tuple too long : " + my_tuple)
    assert len(my_tuple) < 256
    return my_tuple


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


# def run_program_simple(*args, **kwargs):
#     """ simple way to run a command ignoring stderr and stdout
#     """
#     output = os.system(" ".join(*args) + " >/dev/null 2>&1", **kwargs)
#     if output != 0:
#         raise RuntimeError("failed to execute " + " ".join(*args))


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


def float_to_int(param):
    """help to convert param to int
    """
    return str(int(float(param)))


def get_pixel_format_for_metric_computation(subsampling):
    """ helper to set pixel format from subsampling, assuming full range,
        for metric computation
    """
    pixel_format = None
    if subsampling == '420':
        pixel_format = 'yuvj420p'
    elif subsampling == '444':
        pixel_format = 'yuvj444p'
    elif subsampling == '444u':
        pixel_format = 'yuvj444p'
    else:
        raise RuntimeError('Unsupported subsampling ' + subsampling)
    return pixel_format


def get_pixel_format_for_encoding(subsampling):
    """ helper to set pixel format from subsampling, assuming full range,
        for converting source to yuv prior to encoding
    """
    pixel_format = None
    if subsampling == '420':
        pixel_format = 'yuvj420p'
    elif subsampling == '444':
        pixel_format = 'yuvj444p'
    elif subsampling == '444u':
        pixel_format = 'yuvj420p'
    else:
        raise RuntimeError('Unsupported subsampling ' + subsampling)
    return pixel_format


def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value

if __name__ == '__main__':
    pass
