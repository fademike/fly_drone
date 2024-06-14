
SUBDIRS=stm32f103_BL
SUBDIRS+=stm32f103_FlyBoard

%: 
	@for dir in $(SUBDIRS); \
	do \
		$(MAKE) -C $${dir} $@ || exit $$?; \
#		cp $${dir}/out/$${dir}.bin ./; \
	done

