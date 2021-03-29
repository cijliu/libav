OUT_APP         = test
PWD = $(shell pwd)/../../
INCLUDE_PATH = $(PWD)/build/include/
INCLUDE = -I$(INCLUDE_PATH)libavutil/ -I$(INCLUDE_PATH)libavdevice/ \
			-I$(INCLUDE_PATH)libavcodec/ -I$(INCLUDE_PATH)libswresample/ \
			-I$(INCLUDE_PATH)libavfilter/ -I$(INCLUDE_PATH)libavformat/ \
			-I$(INCLUDE_PATH)libswscale/ -I$(INCLUDE_PATH)/
 
#FFMPEG_LIBS = -lavformat -lavutil -lavdevice -lavcodec -lavresample -lavfilter -lswscale 

FFMPEG_LIBS = -L$(PWD)/build/lib -lavcodec -lavdevice -lavfilter -lavformat -lavresample -lavutil -lswscale  \
			  -L$(PWD)/libavdevice/platform/hisilicon/hi3516ev200	-lmpp -ldl
SDL_LIBS    =
LIBS        = $(FFMPEG_LIBS)$(SDL_LIBS)
 
COMPILE_OPTS = $(INCLUDE)
C              = c
OBJ          = o
CROSS_COMPILER =arm-himix100-linux-
C_COMPILER   = $(CROSS_COMPILER)gcc
C_FLAGS      = $(COMPILE_OPTS) $(CPPFLAGS) $(CFLAGS)
 
LINK          = $(CROSS_COMPILER)gcc 
LINK_OPTS    =  -lm  -lpthread  
LINK_OBJ     = capture.c
 
.$(C).$(OBJ):
	$(C_COMPILER) -c $(C_FLAGS) $<
 
 
$(OUT_APP): clean
	$(LINK) $(INCLUDE) -o $@  $(LINK_OBJ)  $(LIBS) $(LINK_OPTS)
	
 
clean:
	-rm -rf *.$(OBJ) $(OUT_APP) core *.core *~  *.jpeg
