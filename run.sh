rm -rf /image_test/*
cd /image_test/
python3 /code/script_compress_parallel.py
tree -C /image_test/
rm -rf /code/image_test/
cp -r /image_test/ /code/
