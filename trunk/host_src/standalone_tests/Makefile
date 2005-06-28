#set this to 1 if you want to build under Windows:
WIN32=0
# set this to 1 if you are under unix but want windows binaries
CROSSCOMPILE=0
#


USE_CCACHE=0
USE_DEBUG=0

CXX=g++
CC=gcc

CFLAGS=-I. -g -pedantic -Wall
LDFLAGS=
EXEC_EXT=
LIB_EXT=.o
COMPILERPREFIX=

LIB_PATH=../lib
VPATH+=$(LIB_PATH)

ifeq ($(CROSSCOMPILE),1)
  COMPILERPREFIX=/opt/xmingw/bin/i386-mingw32msvc-
endif

ifeq ($(USE_CCACHE),1)
  CXX:=ccache $(CXX)
  CC:=ccache $(CC)
endif


ifeq ($(WIN32),1)
  CFLAGS+=-D__WIN32__
  EXEC_EXT=.exe
  LIB_EXT=.lib
endif

ifeq ($(USE_DEBUG),1)
  CFLAGS+=-DDEBUG
endif


#ifeq ($(origin COMPILERPREFIX), undefined)
ifdef COMPILERPREFIX
  CXX:=$(COMPILERPREFIX)$(CXX)
  CC:=$(COMPILERPREFIX)$(CC)
endif

PROGS=phcc_reset phcc_doa_raw phcc_getkeymap phcc_watch_talking phcc_lcd_demo

phcc_reset_OBJS=phcc_reset PHCC_Serial MB_Opt
phcc_reset_LIBS=

phcc_doa_raw_OBJS=phcc_doa_raw PHCC_Serial MB_Opt
phcc_doa_raw_LIBS=

phcc_getkeymap_OBJS=phcc_getkeymap PHCC_Serial MB_Opt
phcc_getkeymap_LIBS=

phcc_watch_talking_OBJS=phcc_watch_talking PHCC_Serial MB_Opt
phcc_watch_talking_LIBS=

phcc_lcd_demo_OBJS=phcc_lcd_demo PHCC_Serial MB_Opt
phcc_lcd_demo_LIBS=


HEADERS=PHCC_Serial.h MB_Opt.h
INC_PATH=$(VPATH:%=-I%)
CFLAGS+=$(INC_PATH)

##########################################################################
all build: $(PROGS)

define PROGRAM_template
$(1): $$($(1)_OBJ) $$($(1)_OBJS:%=%$$(LIB_EXT))
	$$(CXX) $$(LDFLAGS) -o $$@$$(EXEC_EXT) $$^
ALL_OBJS += $$($(1)_OBJS:%=%$$(LIB_EXT))
endef


$(foreach prog,$(PROGS),$(eval $(call PROGRAM_template,$(prog))))


%.lib : %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c -o $@ $<

%.lib : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.o : %.cpp $(HEADERS)
	$(CXX) $(CFLAGS) -c -o $@ $<

%.o : %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

%.h:
	echo "header file dependency: $<"

clean:
	rm -f core* $(PROGS) $(PROGS:%=%.exe) *.o *.lib $(ALL_OBJS)

doc-clean:
	rm -f *.pdf *.ps *.dvi *.aux *.log

dep depend install:
