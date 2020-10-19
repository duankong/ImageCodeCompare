import subprocess


def run_program(LOGGER, *args, **kwargs):
    """ run command using subprocess.check_output, collect stdout and stderr together and return
    """

    kwargs.setdefault("stderr", subprocess.STDOUT)
    kwargs.setdefault("shell", True)
    # kwargs.setdefault("universal_newlines", True)
    try:
        output = subprocess.check_output(" ".join(*args), **kwargs)
        return decode(output)

    except subprocess.CalledProcessError as e:
        LOGGER.error("***** ATTENTION : subprocess call crashed: %s\n%s", args, e.output)
        raise


def run_program_no_LOGGER(*args, **kwargs):
    """ run command using subprocess.check_output, collect stdout and stderr together and return
    """

    kwargs.setdefault("stderr", subprocess.STDOUT)
    kwargs.setdefault("shell", True)
    # kwargs.setdefault("universal_newlines", True)
    try:
        output = subprocess.check_output(" ".join(*args), **kwargs)
        return decode(output)

    except subprocess.CalledProcessError as e:
        print("***** ATTENTION : subprocess call crashed: %s\n%s", args, e.output)
        raise


def decode(value):
    """ Convert bytes to string, if needed
    """
    if isinstance(value, bytes):
        return value.decode("utf-8")
    return value


def my_exec(LOGGER, cmd):
    """ helper to choose method for running commands
    """
    return run_program(LOGGER, cmd)


if __name__ == '__main__':
    pass
