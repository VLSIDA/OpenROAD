TAG_DATE := $(shell date +%F)

build:
	docker build -t vlsida/openroad-ubuntu:${TAG_DATE} -f Dockerfile . | tee -i openroad-ubuntu.log
	docker tag  vlsida/openroad-ubuntu:${TAG_DATE} vlsida/openroad-ubuntu:latest
.PHONY: build

rebuild:
	docker build --no-cache -t vlsida/openroad-ubuntu:${TAG_DATE} -f Dockerfile . | tee -i openroad-ubuntu.log
	docker tag  vlsida/openroad-ubuntu:${TAG_DATE} vlsida/openroad-ubuntu:latest
.PHONY: rebuild

mount:
	@docker run -it \
		-v .:/openroad \
		-v $(HOME):$(HOME)\
		-v $(PDK_ROOT):/pdk \
		-e PDK_ROOT=/pdk \
		-w /openroad \
        vlsida/openroad-ubuntu:latest
.PHONY: mount
