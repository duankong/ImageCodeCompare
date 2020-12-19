#！/bin/bash
## init
#sh /code/init.sh


## COMPRESSION
cd /code/TestLog/X_Chest_runs/
python3 /code/Learning/Duankong/2020_Doing/ImageCodeCompare/a_compress_image_parallel_script.py  \
--image_path=/code/Learning/Duankong/2020_Doing/prepare/TestData/Test1_lossless_8bit/X_Chest/souceImage  \
--work_dir=/code/TestLog/X_Chest_runs/  \
--func_choice=lossless  \
-mc=True \


