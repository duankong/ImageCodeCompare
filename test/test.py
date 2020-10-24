import os
import shutil
import subprocess

# test root
test_result_root = '/code/test/result'
test_image_root = '/code/test/images'

# project info
project_root = '/code'
project = (["a_compress_image_parallel_script.py",
            "a_compress_8bit_video_parallel_script.py"])


def test_run():
    indx = 0

    for depth in ["8", "16"]:
        for model in ["auto", "customize", "lossless"]:
            image_path = "{}/test_{}bit_image".format(test_image_root, depth)
            result_path = "{}/runs_image_{}_{}/".format(test_result_root, depth, model)
            make_dirs(result_path)
            cmd = ["python3", '{}/{}'.format(project_root, project[0]),
                   '--image_path', image_path,
                   '--work_dir', result_path,
                   '--func_choice', model,
                   ]
            print("[ ** ] Run Image depth = {:<2} model = {} ".format(depth, model))
            run_program_no_LOGGER(cmd, cwd=result_path)

    for depth in ["8"]:
        for model in ["lossy", "lossless"]:
            image_path = "{}/test_{}bit_video".format(test_image_root, depth)
            result_path = "{}/runs_video_{}_{}/".format(test_result_root, depth, model)
            make_dirs(result_path)
            cmd = ["python3", '{}/{}'.format(project_root, project[1]),
                   '--image_path', image_path,
                   '--work_dir', result_path,
                   ]
            batchfile, yuv_files = init_yuv_files(result_path)
            cmd[2:2] = ['--batch_image_dir={}'.format(batchfile)]
            cmd[2:2] = ['--yuv_dir={}'.format(yuv_files)]
            cmd[2:2] = ['--prepare_yuv={}'.format("True")]

            if model == 'lossless':
                cmd[2:2] = ['--lossless=True']
            else:
                cmd[2:2] = ['--lossless=False']
            print("[ ** ] Run Video depth = {:<2} model = {} ".format(depth, model))
            run_program_no_LOGGER(cmd, cwd=result_path)


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


def make_dirs(target_path):
    if os.path.exists(target_path):
        shutil.rmtree(target_path)
    os.makedirs(target_path)


def init_yuv_files(result_path):
    batchfile = os.path.join(result_path, 'batch_image')
    make_dirs(batchfile)
    yuv_files = os.path.join(result_path, 'yuv_files')
    make_dirs(yuv_files)
    return batchfile, yuv_files


if __name__ == '__main__':
    test_run()
