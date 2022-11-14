

build:
	dt-build_utils-cli aido-container-build --use-branch daffy --ignore-dirty --ignore-untagged --push --buildx --platforms linux/amd64,linux/arm64


submit-bea:
	dts challenges submit --impersonate 1639 --challenge 'aido-LF*' --retire-same-label

submit:
	dts challenges submit
