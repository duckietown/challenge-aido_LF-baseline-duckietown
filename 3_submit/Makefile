AIDO_REGISTRY ?= docker.io
PIP_INDEX_URL ?= https://pypi.org/simple

repo=challenge-aido_lf-baseline-duckietown
# repo=$(shell basename -s .git `git config --get remote.origin.url`)
branch=$(shell git rev-parse --abbrev-ref HEAD)
tag=$(AIDO_REGISTRY)/duckietown/$(repo):$(branch)


build_options =  \
	--build-arg  AIDO_REGISTRY=$(AIDO_REGISTRY) \
	 --build-arg  PIP_INDEX_URL=$(PIP_INDEX_URL)


update-reqs:
	pur --index-url $(PIP_INDEX_URL) -r requirements.txt -f -m '*' -o requirements.resolved
	aido-update-reqs requirements.resolved

build: update-reqs
	docker build --pull -t $(tag) .

build-no-cache: update-reqs
	docker build -t $(tag)  --no-cache .

push: build
	docker push $(tag)

submit-bea: update-reqs
	dts challenges submit --impersonate 1639 --challenge all --retire-same-label

submit: update-reqs
	dts challenges submit
