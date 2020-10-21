###############################################################
# copyright 2016-2016, Jiang GUO, Yimin ZHOU, UESTC           #
###############################################################

INCLUDE_DIRECTORIES( inc )

ADD_LIBRARY(lcommon STATIC
inc/block_info.h
inc/ComAdaptiveLoopFilter.h
inc/commonStructures.h
inc/commonVariables.h
inc/contributors.h
inc/defines.h
inc/inter-prediction.h
inc/intra-prediction.h
inc/loop-filter.h
inc/md5.h
inc/memalloc.h
inc/transform.h
src/block_info.c
src/ComAdaptiveLoopFilter.c
src/inter-prediction.c
src/intra-prediction.c
src/loop-filter.c
src/md5.c
src/memalloc.c
src/transform.c
)

# SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../../lib)
