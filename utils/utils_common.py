import os
import errno
import ntpath
import glob



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


def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def run_program_simple(*args, **kwargs):
    """ simple way to run a command ignoring stderr and stdout
    """
    output = os.system(" ".join(*args) + " >/dev/null 2>&1", **kwargs)
    if output != 0:
        raise RuntimeError("failed to execute " + " ".join(*args))


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


if __name__ == '__main__':
    pass
