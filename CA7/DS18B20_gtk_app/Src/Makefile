BIN_DIR=./bin/
BIN_DIR_DESKTOP=./bin_desktop/
 
LIB_SRCS =  eok_analoggauge.c
SRCS =	ds18b20-gtk-app.c

LIBS = ${foreach src,$(LIB_SRCS), $(patsubst %.c,%.o, $(src))}
LIBS_DESKTOP = ${foreach src,$(LIB_SRCS), $(patsubst %.c,%, $(src))}

PROGS = ${foreach src,$(SRCS), $(patsubst %.c,%, $(src))}
PROGS_DESKTOP = ${PROGS}
# Add / change option in CFLAGS and LDFLAGS
CFLAGS += -Wall -I../Inc $(shell pkg-config --cflags gtk4)
LDFLAGS += -lm $(shell pkg-config --libs gtk4)

LIBSFLAGS = ${foreach src,$(LIB_SRCS), $(patsubst %.c, ./bin/%.o , $(src))}
LIBSFLAGS_DESKTOP = ${foreach src,$(LIB_SRCS), $(patsubst %.c, ./bin_desktop/%_desktop.o , $(src))}
#CFLAGS += -Wall $(shell pkg-config --cflags gtk+-3.0)
#LDFLAGS += $(shell pkg-config --libs gtk+-3.0)


CLEANFILES = ${foreach obj,$(PROGS),$(obj) $(obj).o}
CLEANFILES_DESKTOP = ${foreach obj,$(PROGS),$(obj)_desktop $(obj)_desktop.o} 
CLEANFILES_LIB = ${foreach obj,$(LIBS), $(obj).o} ./bin
CLEANFILES_LIB_DESKTOP = ${foreach obj,$(LIBS), $(obj)_desktop.o} ./bin_desktop
.PHONY : all
all : $(PROGS)

${BIN_DIR} :
	mkdir ./bin
	
${BIN_DIR_DESKTOP} :
	mkdir ./bin_desktop
				

${LIBS}: ${BIN_DIR} ${LIB_SRCS}
	${foreach obj,$(LIBS), $(CC) -c $(obj).c -o ./bin/$(obj).o $(CFLAGS) $(LDFLAGS);}
			
desktop: ${SRCS} ${LIB_SRCS} ${BIN_DIR_DESKTOP}
	${foreach obj,$(LIBS_DESKTOP), gcc -c $(obj).c -o ./bin_desktop/$(obj)_desktop.o $(CFLAGS) $(LDFLAGS);}
	${foreach obj,$(PROGS_DESKTOP), gcc -o $(obj)_desktop $(obj).c ${LIBSFLAGS_DESKTOP} $(CFLAGS) $(LDFLAGS) ;}
	
$(PROGS): ${SRCS} ${LIBS}
	${foreach obj,$(PROGS), $(CC) -o $(obj) $(obj).c ${LIBSFLAGS} $(CFLAGS) $(LDFLAGS) ;}
	
clean:
	rm -f $(CLEANFILES) $(CLEANFILES_DESKTOP) $(CLEANFILES_LIB) $(CLEANFILES_LIB_DESKTOP)

