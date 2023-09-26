# VEXcode mkrules.mk 2019_03_26_01

# compile C files
$(BUILD)/%.o: %.c $(SRC_H)
	$(Q)$(MKDIR)
	$(ECHO) "CC  $<"
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<
	
# compile C++ files
$(BUILD)/%.o: %.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<
	
# create executable 
$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LINK) $(LNK_FLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

# create binary 
$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(Q)$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# create archive
$(BUILD)/$(PROJECTLIB).a: $(OBJ)
	$(Q)$(ARCH) $(ARCH_FLAGS) $@ $^

# upload to brain using ProsPlus
upload: $(BUILD)/$(PROJECT).bin
	echo $(VEX_DEV_HOME)/Play/$(PROJECT)/$(BUILD)/$(PROJECT).bin
	call "$(VEX_DEV_HOME)\ProsPlus\run_uploader.bat" 1 $(PROJECT) $(VEX_ICON) "$(VEX_DEV_HOME)\Play\$(PROJECT)\$(BUILD)\$(PROJECT).bin"
	echo Done
# clean project

all: clean upload
	echo Build it all

clean:
	$(info clean project)
	$(Q)$(CLEAN)
