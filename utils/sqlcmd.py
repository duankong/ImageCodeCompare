def get_insert_command():
    """ helper to get DB insert command
    """
    return '''INSERT INTO ENCODES (ID,SOURCE,WIDTH,HEIGHT,CODEC,CODEC_PARAM,
    TEMP_FOLDER,TARGET_METRIC,TARGET_VALUE,
    VMAF,SSIM,MS_SSIM,VIF,MSE_Y,MSE_U,MSE_V,MSE_AVG,PSNR_Y,PSNR_U,PSNR_V,PSNR_AVG,ADM2,
    SUB_SAMPLING,FILE_SIZE_BYTES,ENCODED_FILE) 
    VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)'''


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
         VMAF             REAL    NOT NULL,
         SSIM             REAL    NOT NULL,
         MS_SSIM          REAL    NOT NULL,
         VIF              REAL    NOT NULL,
         MSE_Y            REAL    NOT NULL,
         MSE_U            REAL    NOT NULL,
         MSE_V            REAL    NOT NULL,
         MSE_AVG          REAL    NOT NULL,
         ADM2             REAL    NOT NULL,
         PSNR_Y           REAL    NOT NULL,
         PSNR_U           REAL    NOT NULL,
         PSNR_V           REAL    NOT NULL,
         PSNR_AVG         REAL    NOT NULL,
         SUB_SAMPLING     TEXT    NOT NULL,
         FILE_SIZE_BYTES  REAL    NOT NULL,
         ENCODED_FILE     TEXT    NOT NULL);'''


if __name__ == '__main__':
    pass
