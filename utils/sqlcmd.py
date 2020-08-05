def get_insert_command():
    """ helper to get DB insert command
    """
    return '''INSERT INTO ENCODES (ID,SOURCE,WIDTH,HEIGHT,CODEC,CODEC_PARAM,TEMP_FOLDER,TARGET_METRIC,TARGET_VALUE,
    SSIM,MSE,PSNR,FILE_SIZE_BYTES,ENCODED_FILE) 
    VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)'''


def get_create_table_command():
    """ helper to get create table command
    """
    return '''CREATE TABLE ENCODES
         (ID TEXT PRIMARY KEY     NOT NULL,
         SOURCE           TEXT    NOT NULL,
         WIDTH            INT     NOT NULL,
         HEIGHT           INT     NOT NULL,
         CODEC            TEXT    NOT NULL,
         CODEC_PARAM      REAL    NOT NULL,
         TEMP_FOLDER      TEXT    NOT NULL,
         TARGET_METRIC    TEXT    NOT NULL,
         TARGET_VALUE     REAL    NOT NULL,
         SSIM             REAL    NOT NULL,
         MSE              REAL    NOT NULL,
         PSNR             REAL    NOT NULL,
         FILE_SIZE_BYTES  REAL    NOT NULL,
         ENCODED_FILE     TEXT    NOT NULL);'''


if __name__ == '__main__':
    pass
