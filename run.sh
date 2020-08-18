#！/bin/bash

## init
rm -rf /image_test/*
rm -rf /code/image_test/
rm -rf /code/encoding_result_*
## PARAMETER
metric="psnr_avg"
db_file_name="encoding_results_${metric}.db"

## COMPRESSION
cd /image_test/
python3 /code/script_compress_parallel.py \
--metric ${metric} \
--db_file_name=${db_file_name} \
#cp -f  /code/image_test/encoding_results* /code/

## COMPUTER_BD_RATES 
#python3 /code/computer_BD_rates.py --db_file_name=${db_file_name} \

## ANALYZE ENCODING RESULT
#python3 /code/analyze_encoding_results.py --metric ${metric} --db_file_name=${db_file_name} \

#cp -r /image_test/ /code/