

pod_dirs:=$(shell find distro/pods -maxdepth 2 -name pod-build -print0)


default:
	@if [ -z $(pod_dirs) ]; then \
		echo \\nOops!  Please specify a target:  $(MAKE)  \<drc\|drake-distro\|superbuild\>\\n; \
	else \
		for d in $(pod_dirs); do \
			$(MAKE) -C `dirname "$$d"`; \
		done; \
	fi

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
