SRCS =	ds18b20-gtk-app.c

PROGS = ${foreach src,$(SRCS), $(patsubst %.c,%, $(src))}
PROGS_DESKTOP = ${PROGS}
# Add / change option in CFLAGS and LDFLAGS
CFLAGS += -Wall $(shell pkg-config --cflags gtk4)
LDFLAGS += -lm $(shell pkg-config --libs gtk4)

#CFLAGS += -Wall $(shell pkg-config --cflags gtk+-3.0)
#LDFLAGS += $(shell pkg-config --libs gtk+-3.0)


CLEANFILES = ${foreach obj,$(PROGS),$(obj) $(obj).o}
CLEANFILES_DESKTOP = ${foreach obj,$(PROGS),$(obj)_desktop $(obj)_desktop.o} 

.PHONY : all
all : $(PROGS)
	
desktop: ${SRCS}
	${foreach obj,$(PROGS_DESKTOP), gcc -o $(obj)_desktop $(obj).c $(CFLAGS) $(LDFLAGS);}
	
$(PROGS) : ${SRCS}
	${foreach obj,$(PROGS), $(CC) -o $(obj) $(obj).c $(CFLAGS) $(LDFLAGS);}
	
clean:
	rm -f $(CLEANFILES) $(CLEANFILES_DESKTOP)


