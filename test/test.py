import os
import shutil
import subprocess

# test root
test_result_root = '/code/test/result'
test_image_root = '/code/test/images'
# result root
result_files = (["result_8bit_image",
                 "result_8bit_video",
                 "result_16bit_image"])
result_files_lossless = (["result_8bit_image_lossless",
                          "result_8bit_video_lossless",
                          "result_16bit_image_lossless"])
# image root
image_files = (["test_8bit_image",
                "test_8bit_video",
                "test_16bit_image"])
# project info
project_root = '/code/'
project = (["a_compress_8bit_image_parallel_script.py",
            "a_compress_8bit_video_parallel_script.py",
            "a_compress_16bit_image_parallel_script.py"])


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


def init(result_files):
    for num, result_file in enumerate(result_files):
        target_path = os.path.join(test_result_root, result_file)
        make_dirs(target_path)


def test_run():
    init(result_files)
    init(result_files_lossless)
    indx = 0
    for num, pj in enumerate(project):
        image_path = '{}/{}'.format(test_image_root, image_files[num])
        # lossy
        result_path = '{}/{}/'.format(test_result_root, result_files[num])
        cmd = ['python3', '{}{}'.format(project_root, pj), '--image_path', image_path, '--lossless=False', '--work_dir',
               result_path]
        if num == 1:
            batchfile, yuv_files = init_yuv_files(result_path)
            cmd[2:2] = ['--batch_image_dir={}'.format(batchfile)]
            cmd[2:2] = ['--yuv_dir={}'.format(yuv_files)]
            cmd[2:2] = ['--prepare_yuv={}'.format("True")]
        indx += 1
        print("[ {} ][ {}/8 ] run {} in lossy".format(num, indx, pj))
        run_program_no_LOGGER(cmd, cwd=result_path)
        # lossless
        result_path = '{}/{}/'.format(test_result_root, result_files_lossless[num])
        cmd = ['python3', '{}{}'.format(project_root, pj), '--image_path', image_path, '--work_dir', result_path,
               '--lossless=True']
        if num == 1:
            batchfile, yuv_files = init_yuv_files(result_path)
            cmd[2:2] = ['--batch_image_dir={}'.format(batchfile)]
            cmd[2:2] = ['--yuv_dir={}'.format(yuv_files)]
            cmd[2:2] = ['--prepare_yuv={}'.format("True")]
        indx += 1
        print("[ {} ][ {}/8 ] run {} in lossless".format(num, indx, pj))
        run_program_no_LOGGER(cmd, cwd=result_path)

    print("[    ] Done! ")


if __name__ == '__main__':
    test_run()
