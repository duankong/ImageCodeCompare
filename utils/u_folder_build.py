import shutil
import os


def split_file(image):
    (filepath, tempfilename) = os.path.split(image)
    filename, extension = os.path.splitext(tempfilename)
    return filepath, filename, extension


def remove_files(path):
    os.remove(path)


def make_dirs(target_path):
    if os.path.exists(target_path):
        shutil.rmtree(target_path)
    os.makedirs(target_path)


def check_dirs(target_path):
    if not os.path.exists(target_path):
        os.makedirs(target_path)


def listdir_full_path(directory):
    """ like os.listdir(), but returns full absolute paths
    """
    for f in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, f)):
            yield os.path.abspath(os.path.join(directory, f))


def get_filename_with_temp_folder(temp_folder, filename):
    """ helper to get filename with temp folder
    """
    return os.path.join(temp_folder, filename)


if __name__ == '__main__':
    pass
