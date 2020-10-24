import os
import shutil
import subprocess

# test root
test_result_root = '/code/test/result'
test_image_root = '/code/test/images'

# project info
project_root = '/code'
project = (["a_compress_image_parallel_script.py", "a_compress_8bit_video_parallel_script.py"])


def run_image_compres():
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


def run_video_compres():
    for depth in ["8"]:
        for model in ["auto", "customize", "lossless"]:
            image_path = "{}/test_{}bit_video".format(test_image_root, depth)
            result_path = "{}/runs_video_{}_{}/".format(test_result_root, depth, model)
            batchfile, yuv_files = init_yuv_files(result_path)
            make_dirs(result_path)
            cmd = ["python3", '{}/{}'.format(project_root, project[1]),
                   '--image_path', image_path,
                   '--func_choice', model,
                   '--batch_image_dir={}'.format(batchfile),
                   '--yuv_dir={}'.format(yuv_files),
                   '--prepare_yuv={}'.format("True"),
                   '--work_dir', result_path
                   ]

            print("[ ** ] Run Video depth = {:<2} model = {} ".format(depth, model))
            run_program_no_LOGGER(cmd, cwd=result_path)


def run_image_video_compress(choice):
    if choice == 'image':
        run_image_compres()
    elif choice == 'video':
        run_video_compres()
    elif choice == 'image_video':
        run_image_compres()
        run_video_compres()
    else:
        print("[**run_image_video_compress**] Not support choice with {}".format(choice))


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
    choice = ['image', 'video', 'image_video']
    run_image_video_compress(choice=choice[1])
