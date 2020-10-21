###############################################################
# copyright 2016-2016, Jiang GUO, Yimin ZHOU, UESTC           #
###############################################################

INCLUDE_DIRECTORIES( inc
../lcommon/inc)

SET(DIR_SRCS
inc/adaptiveQP.h
inc/AEC.h
inc/biariencode.h
inc/bitstream.h
inc/block.h
inc/codingUnit.h
inc/configfile.h
inc/EncAdaptiveLoopFilter.h
inc/fast_me.h
inc/global.h
inc/header.h
inc/image.h
inc/mv-search.h
inc/pos_info.h
inc/ratecontrol.h
inc/rdopt_coding_state.h
inc/rdoq.h
inc/refbuf.h
inc/tdrdo.h
inc/vlc.h
inc/wquant.h
src/AEC.c
src/background.c
src/biariencode.c
src/bitstream.c
src/block.c
src/codingUnit.c
src/configfile.c
src/context_ini.c
src/EncAdaptiveLoopFilter.c
src/fast_me.c
src/golomb.c
src/header.c
src/image.c
src/lencod.c
src/loopfilter.c
src/mv-search.c
src/pos_info.c
src/ratecontrol.c
src/rdopt.c
src/rdopt_coding_state.c
src/rdoq.c
src/refbuf.c
src/slice.c
src/tdrdo.c
src/vlc.c
src/wquant.c
)

ADD_EXECUTABLE(lencod ../bin/encoder_ai.cfg ../bin/encoder_ldp.cfg ../bin/encoder_ra.cfg ${DIR_SRCS})
TARGET_LINK_LIBRARIES(lencod lcommon)
