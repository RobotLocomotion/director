
default:
	@echo \\nOops!  Please specify a target:  $(MAKE)  \<drc\|drake-distro\|superbuild\>\\n

help: default

drc:
	$(MAKE) -C distro/pods/drc

drake-distro:
	$(MAKE) -C distro/pods/drake-distro

superbuild:
	@mkdir -p build
	@cd build && cmake ../distro/superbuild
	$(MAKE) -C build

clean:
	@for pod in `ls distro/pods`; do $(MAKE) -C distro/pods/$$pod clean; done
