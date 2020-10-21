###
###     Makefile for AVS encoder
###
###     generated for UNIX/LINUX environments



NAME=   lencod

### include debug information: 1=yes, 0=no
DBG?= 0

DEPEND= dependencies

LENDIR= ../../../source/lencod
BINDIR= ../../../source/bin
INCDIR= $(LENDIR)/inc
SRCDIR= $(LENDIR)/src
OBJDIR= ./obj

ADDSRCDIR= $(LENDIR)/../lcommon/src
ADDINCDIR= $(LENDIR)/../lcommon/inc

CC=     $(shell which gcc)
#CC = /rtkoss/gcc/4.1.2/rhel6/x86_64/bin/gcc

LIBS=   -lm
FLAGS=  -ffloat-store -Wall -I$(INCDIR) -I$(ADDINCDIR) -D_FILE_OFFSET_BITS=64

ifeq ($(DBG),1)
SUFFIX= .dbg
FLAGS+= -g -O0
else
SUFFIX=
FLAGS+= -O2
endif

OBJSUF= .o$(SUFFIX)

SRC=    $(wildcard $(SRCDIR)/*.c) 
ADDSRC= $(wildcard $(ADDSRCDIR)/*.c)
OBJ=    $(SRC:$(SRCDIR)/%.c=$(OBJDIR)/%.o$(SUFFIX)) $(ADDSRC:$(ADDSRCDIR)/%.c=$(OBJDIR)/%.o$(SUFFIX)) 
BIN=    $(BINDIR)/$(NAME)$(SUFFIX).exe


default: depend bin tags

dependencies:
	@echo "" >dependencies	
clean:
clean:
	@echo remove all objects
	@rm -f $(OBJDIR)/*
	@rm -f $(BIN)
tags:
	@echo update tag table
	@ctags $(LENDIR)/inc/*.h $(LENDIR)/src/*.c

bin:    $(OBJ)
	@echo
	@echo 'creating binary "$(BIN)"'
	@$(CC) -o $(BIN) $(OBJ) $(LIBS)
	@echo '... done'
	@echo

depend:
	@echo
	@echo 'checking dependencies'
	@echo 'Making the obj directory'	
	@mkdir -p $(OBJDIR)
	@$(SHELL) -ec '$(CC) -MM $(CFLAGS) -I$(INCDIR) -I$(ADDINCDIR) $(SRC) $(ADDSRC)                  \
         | sed '\''s@\(.*\)\.o[ :]@$(OBJDIR)/\1.o$(SUFFIX):@g'\''               \
         >$(DEPEND)'
	@echo

$(OBJDIR)/%.o$(SUFFIX): $(SRCDIR)/%.c
	@echo 'compiling object file "$@" ...'
	@$(CC) -c -o $@ $(FLAGS) $<

$(OBJDIR)/%.o$(SUFFIX): $(ADDSRCDIR)/%.c
	@echo 'compiling object file "$@" ...'
	@$(CC) -c -o $@ $(FLAGS) $<

#include $(DEPEND)

