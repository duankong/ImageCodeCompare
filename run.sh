#！/bin/bash

## init
sh /code/init.sh
## PARAMETER
metric="ssim"
db_file_name="encoding_results_${metric}.db"

## COMPRESSION
cd /image_test/
python3 /code/script_compress_parallel.py \
--num_process 6 \
#--metric ${metric} \
#--db_file_name=${db_file_name} \

## COMPUTER_BD_RATES 
#python3 /code/computer_BD_rates.py --db_file_name=${db_file_name} \

## ANALYZE ENCODING RESULT
#python3 /code/analyze_encoding_results.py --metric ${metric} --db_file_name=${db_file_name} \

cp -r /image_test/ /code/