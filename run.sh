#！/bin/bash
## init
sh /code/init.sh
## PARAMETER
metric="ssim"
db_file_name="encoding_results_${metric}.db"

## COMPRESSION
cd /code/runs/
python3 /code/a_compress_image_parallel_script.py   \
--func_choice='lossless' \

