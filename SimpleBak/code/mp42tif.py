import cv2
import os
from libtiff import TIFF
from mpi4py import MPI


def video2images(videodeopath):
    (filepath, tempfilename) = os.path.split(videodeopath)
    filename, extension = os.path.splitext(tempfilename)
    video = cv2.VideoCapture(videodeopath)
    tiffname = os.path.join(filepath, '{}.tif'.format(filename))
    out_tiff = TIFF.open(tiffname, mode='w')
    for index in range(512):
        ret, frame = video.read()
        out_tiff.write_image(frame, compression=None)
        out_tiff.SetDirectory(index)
        out_tiff.writedirectory()
    video.release()
    out_tiff.close()


def get_task_list():
    # 得到所有mp4文件的绝对路径
    task_list = []
    return task_list


def fun_c():
    taskslist = get_task_list()
    comm = MPI.COMM_WORLD
    comm_rank = comm.Get_rank()
    comm_size = comm.Get_size()

    # point to point communication
    if comm_rank == 0:
        for i in range(comm_size - 1):
            comm.send(0, i + 1)

        for n, task in enumerate(taskslist):
            task_comm_rank = comm.recv(source=MPI.ANY_SOURCE)
            comm.send(task, dest=task_comm_rank)

        for n in range(comm_size - 1):
            comm.send(-1, dest=n + 1)
    else:
        comm.recv(source=0)
        comm.send(comm_rank, dest=0)
        while True:
            task = comm.recv(source=0)
            if task != -1:
                video2images(task)
                print("[** rank {}] Ireceived:{}".format(comm_rank, task))
                comm.send(comm_rank, dest=0)

            else:
                return


if __name__ == '__main__':
    fun_c()
