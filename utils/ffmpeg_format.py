"""
----------------------------------------------
PIXEL_FORMAT
----------------------------------------------
"""


def get_pixel_format(subsampling, depth):
    """ helper to set pixel format from subsampling, assuming full range,
        for converting source to yuv prior to encoding
    """
    pixel_format = None
    if depth == '8':
        if subsampling == '420':
            pixel_format = 'yuvj420p'
        elif subsampling == '444':
            pixel_format = 'yuvj444p'
        else:
            raise RuntimeError('Unsupported subsampling ' + subsampling)
    elif depth == '16':
        if subsampling == '420':
            pixel_format = 'yuv420p16le'
        elif subsampling == '444':
            pixel_format = 'yuv444p16le'
        else:
            raise RuntimeError('Unsupported subsampling ' + subsampling)
    else:
        raise RuntimeError('Unsupported depth ' + depth)
    return pixel_format


if __name__ == '__main__':
    pass
