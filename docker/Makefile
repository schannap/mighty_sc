# at the top of your Makefile
.PHONY: build
build:
	docker build \
	  -t mighty .

.PHONY: build-no-cache
build-no-cache:
	docker build \
	--build-arg DUMMY=`date +%s` -t mighty .

run:
	docker run  -it \
				-v /home/intent/code/mighty_ws/src/mighty_sc:/home/kkondo/code/mighty_ws/src/mighty \
				-w /home/kkondo/code/mighty_ws/src/mighty \
				--env="NVIDIA_DRIVER_CAPABILITIES=all" \
				--privileged \
				-e DISPLAY=${DISPLAY} \
				-e XAUTHORITY=$$XAUTHORITY \
				-v $$XAUTHORITY:$$XAUTHORITY:ro \
				--network=host \
				--env="QT_X11_NO_MITSHM=1" \
				--rm mighty
