
# SUBDIRS=stm32f103_BL
SUBDIRS+=stm32f103_FlyBoard

%: 
	@for dir in $(SUBDIRS); \
	do \
		$(MAKE) -C $${dir} $@ || exit $$?; \
#		cp $${dir}/out/$${dir}.bin ./; \
	done
	
# load: 
# 	@for dir in $(SUBDIRS); \
# 	do \
# 		$(MAKE) -C $${dir} $@ || exit $$?; \
# 	done

# debug: 
# 	$(MAKE) -C $(SUBDIRS) $@ || exit $$?;
	
# clean: 
# #	rm -f ./*.bin
# 	@for dir in $(SUBDIRS); \
# 	do \
# 		$(MAKE) -C $${dir} $@ || exit $$?; \
# 	done
        
        
