import subprocess
import os
from skimage import io, metrics


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


def run_program(*args, **kwargs):
    """ run command using subprocess.check_output, collect stdout and stderr together and return
    """
    kwargs.setdefault("stderr", subprocess.STDOUT)
    kwargs.setdefault("shell", True)

    try:
        output = subprocess.check_output(" ".join(*args), **kwargs)
        return decode(output)

    except subprocess.CalledProcessError as e:
        print("***** ATTENTION : subprocess call crashed: %s\n%s", args, e.output)
        raise


def convert():
    cmd = ['convert', source_ppm, '-interlace', 'plane', '-sampling-factor', '4:4:4', source_yuv]
    my_exec(cmd)


if __name__ == '__main__':
    images = set(listdir_full_path('/code/images'))
    assert len(images) > 0


