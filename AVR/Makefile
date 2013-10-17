MCU = atmega32
F_CPU = 1000000
FORMAT = ihex
TARGET = ATmega32
OBJDIR = obj
BINDIR = bin
SRC = $(TARGET).c
CPPSRC = 
ASRC =
OPT = s
DEBUG = dwarf-2
EXTRAINCDIRS = 
CSTANDARD = -std=gnu99
CDEFS = -DF_CPU=$(F_CPU)UL
ADEFS = -DF_CPU=$(F_CPU)
CPPDEFS = -DF_CPU=$(F_CPU)UL

#---------------- Compiler Options C ----------------
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)
CPPFLAGS = -g$(DEBUG)
CPPFLAGS += $(CPPDEFS)
CPPFLAGS += -O$(OPT)
CPPFLAGS += -funsigned-char
CPPFLAGS += -funsigned-bitfields
CPPFLAGS += -fpack-struct
CPPFLAGS += -fshort-enums
CPPFLAGS += -fno-exceptions
CPPFLAGS += -Wundef
CPPFLAGS += -Wa,-adhlns=$(<:%.cpp=$(OBJDIR)/%.lst)
CPPFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
ASFLAGS = $(ADEFS) -Wa,-adhlns=$(<:%.S=$(OBJDIR)/%.lst),-gstabs,--listing-cont-lines=100
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt
PRINTF_LIB = 
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt
SCANF_LIB = 
MATH_LIB = -lm
EXTRALIBDIRS = 
EXTMEMOPTS =
LDFLAGS = -Wl,-Map=$(BINDIR)/$(TARGET).map,--cref
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(patsubst %,-L%,$(EXTRALIBDIRS))
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)

#---------------- Programming Options (avrdude) ----------------
AVRDUDE_PROGRAMMER = avrispmkII
AVRDUDE_PORT = usb
AVRDUDE_WRITE_FLASH = -U flash:w:$(BINDIR)/$(TARGET).hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(BINDIR)/$(TARGET).eep
AVRDUDE_READ_FLASH = -U flash:r:$(BINDIR)/$(TARGET).hex
AVRDUDE_READ_EEPROM = -U eeprom:r:$(BINDIR)/$(TARGET).eep
AVRDUDE_VERIFY_FLASH = -U flash:v:$(BINDIR)/$(TARGET).hex
AVRDUDE_VERIFY_EEPROM = -U eeprom:v:$(BINDIR)/$(TARGET).eep
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)

#---------------- Debugging Options ----------------
DEBUG_MFREQ = $(F_CPU)
DEBUG_UI = insight
DEBUG_BACKEND = avarice
GDBINIT_FILE = __avr_gdbinit
JTAG_DEV = /dev/com1
DEBUG_PORT = 4242
DEBUG_HOST = localhost
#============================================================================

ifeq ($(OS),Windows_NT)
	CC = avr-gcc
	OBJCOPY = avr-objcopy
	OBJDUMP = avr-objdump
	SIZE = avr-size
	AR = avr-ar rcs
	NM = avr-nm
	AVRDUDE = avrdude
else
	CC = /usr/local/CrossPack-AVR/bin/avr-gcc
	OBJCOPY = /usr/local/CrossPack-AVR/bin/avr-objcopy
	OBJDUMP = /usr/local/CrossPack-AVR/bin/avr-objdump
	SIZE = /usr/local/CrossPack-AVR/bin/avr-size
	AR = /usr/local/CrossPack-AVR/bin/avr-ar rcs
	NM = /usr/local/CrossPack-AVR/bin/avr-nm
	AVRDUDE = /usr/local/CrossPack-AVR/bin/avrdude
endif

SHELL = sh
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp
WINSHELL = cmd
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = begin
MSG_END = end
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_CPP = Compiling C++:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:
MSG_PROG = Programming  AVR:
MSG_VERIFY = Verifying AVR:
MSG_DIV = -------------------------------
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o) $(ASRC:%.S=$(OBJDIR)/%.o) 
LST = $(SRC:%.c=$(OBJDIR)/%.lst) $(CPPSRC:%.cpp=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst) 
GENDEPFLAGS = -MMD -MP -MF $(OBJDIR)/$(@F).d
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_CPPFLAGS = -mmcu=$(MCU) -I. -x c++ $(CPPFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)
all: begin gccversion sizebefore build sizeafter clean_objdir end
#build: elf hex eep lss sym
build: lib
elf: $(BINDIR)/$(TARGET).elf
hex: $(BINDIR)/$(TARGET).hex
eep: $(BINDIR)/$(TARGET).eep
lss: $(BINDIR)/$(TARGET).lss
sym: $(BINDIR)/$(TARGET).sym
lib: $(BINDIR)/$(TARGET).a

begin:
	@echo
	@echo $(MSG_BEGIN)
	@echo $(MSG_DIV)

end:
	@echo $(MSG_DIV)
	@echo $(MSG_END)
	@echo

HEXSIZE = $(SIZE) --target=$(FORMAT) $(BINDIR)/$(TARGET).hex
ELFSIZE = $(SIZE) --mcu=$(MCU) --format=avr $(BINDIR)/$(TARGET).elf

sizebefore:
	@if test -f $(BINDIR)/$(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(BINDIR)/$(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	2>/dev/null; echo; fi

gccversion : 
	@$(CC) --version
 
program: $(BINDIR)/$(TARGET).hex $(BINDIR)/$(TARGET).eep
	@echo
	@echo $(MSG_PROG)
	@echo $(MSG_DIV)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)
	
verify: $(BINDIR)/$(TARGET).hex $(BINDIR)/$(TARGET).eep
	@echo
	@echo $(MSG_VERIFY)
	@echo $(MSG_DIV)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_VERIFY_FLASH) $(AVRDUDE_VERIFY_EEPROM)

gdb-config: 
	@$(REMOVE) $(GDBINIT_FILE)
	@echo define reset >> $(GDBINIT_FILE)
	@echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	@echo end >> $(GDBINIT_FILE)
	@echo file $(BINDIR)/$(TARGET).elf >> $(GDBINIT_FILE)
	@echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	@echo load  >> $(GDBINIT_FILE)
endif
	@echo break main >> $(GDBINIT_FILE)

debug: gdb-config $(BINDIR)/$(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	@echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
	@$(WINSHELL) /c start avarice --jtag $(JTAG_DEV) --erase --program --file \
	$(BINDIR)/$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT)
	@$(WINSHELL) /c pause

else
	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
	@$(WINSHELL) /c start avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)

COFFCONVERT = $(OBJCOPY) --debugging
COFFCONVERT += --change-section-address .data-0x800000
COFFCONVERT += --change-section-address .bss-0x800000
COFFCONVERT += --change-section-address .noinit-0x800000
COFFCONVERT += --change-section-address .eeprom-0x810000

coff: $(BINDIR)/$(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(BINDIR)/$(TARGET).cof
	@echo $(MSG_DIV)
	$(COFFCONVERT) -O coff-avr $< $(BINDIR)/$(TARGET).cof

extcoff: $(BINDIR)/$(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(BINDIR)/$(TARGET).cof
	@echo $(MSG_DIV)
	$(COFFCONVERT) -O coff-ext-avr $< $(BINDIR)/$(TARGET).cof

$(BINDIR)/%.hex: $(BINDIR)/%.elf
	@echo
	@echo $(MSG_FLASH) $@
	@echo $(MSG_DIV)
	$(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse -R .lock $< $@

$(BINDIR)/%.eep: $(BINDIR)/%.elf
	@echo
	@echo $(MSG_EEPROM) $@
	@echo $(MSG_DIV)
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) $< $@ || exit 0

$(BINDIR)/%.lss: $(BINDIR)/%.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	@echo $(MSG_DIV)
	$(OBJDUMP) -h -S -z $< > $@

$(BINDIR)/%.sym: $(BINDIR)/%.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	@echo $(MSG_DIV)
	$(NM) -n $< > $@

.SECONDARY : $(BINDIR)/$(TARGET).a
.PRECIOUS : $(OBJ)
$(BINDIR)/%.a: $(OBJ)
	@echo
	@echo $(MSG_CREATING_LIBRARY) $@
	@echo $(MSG_DIV)
	$(AR) $@ $(OBJ)

.SECONDARY : $(BINDIR)/$(TARGET).elf
.PRECIOUS : $(OBJ)
$(BINDIR)/%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	@echo $(MSG_DIV)
	$(CC) $(ALL_CFLAGS) $^ --output $@ $(LDFLAGS)

$(OBJDIR)/%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	@echo $(MSG_DIV)
	$(CC) -c $(ALL_CFLAGS) $< -o $@
	
$(OBJDIR)/%.o : $(LIBRARIES)$(OTHER_SRC).c
	@echo
	@echo $(MSG_COMPILING) $<
	@echo $(MSG_DIV)
	$(CC) -c $(ALL_CFLAGS) $< -o $@

$(OBJDIR)/%.o : %.cpp
	@echo
	@echo $(MSG_COMPILING_CPP) $<
	@echo $(MSG_DIV)
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@ 

$(BINDIR)/%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@

$(BINDIR)/%.s : %.cpp
	$(CC) -S $(ALL_CPPFLAGS) $< -o $@

$(OBJDIR)/%.o : $(BINDIR)/%.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	@echo $(MSG_DIV)
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

$(BINDIR)/%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@ 

clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	@echo $(MSG_DIV)
	$(REMOVE) $(BINDIR)/$(TARGET).hex
	$(REMOVE) $(BINDIR)/$(TARGET).eep
	$(REMOVE) $(BINDIR)/$(TARGET).cof
	$(REMOVE) $(BINDIR)/$(TARGET).elf
	$(REMOVE) $(BINDIR)/$(TARGET).map
	$(REMOVE) $(BINDIR)/$(TARGET).sym
	$(REMOVE) $(BINDIR)/$(TARGET).lss
	$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.o)
	$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.lst)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRC:.c=.i)
	$(REMOVEDIR) $(BINDIR)
	$(REMOVEDIR) $(OBJDIR)

clean_objdir:
	$(REMOVE) $(BINDIR)/$(TARGET).cof
	$(REMOVE) $(BINDIR)/$(TARGET).elf
	$(REMOVE) $(BINDIR)/$(TARGET).map
	$(REMOVE) $(BINDIR)/$(TARGET).sym
	$(REMOVE) $(BINDIR)/$(TARGET).lss
	$(REMOVEDIR) $(OBJDIR)

$(shell mkdir $(OBJDIR) 2>/dev/null)
$(shell mkdir $(BINDIR) 2>/dev/null)

.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list clean_objdir program debug gdb-config
