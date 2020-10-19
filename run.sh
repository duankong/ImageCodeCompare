#！/bin/bash
## init
sh /code/init.sh
## PARAMETER
metric="ssim"
db_file_name="encoding_results_${metric}.db"

## COMPRESSION
cd /code/image_test/
python3 /code/a_compress_8bit_video_parallel_script.py  \
#--metric ${metric} \
#--db_file_name=${db_file_name} \

## COMPUTER_BD_RATES 
#python3 /code/computer_BD_rates.py --db_file_name=${db_file_name} \

## ANALYZE ENCODING RESULT
#python3 /code/analyze_encoding_results.py --metric ${metric} --db_file_name=${db_file_name} \

#cp -r /image_test/ /code/