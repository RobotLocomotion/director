set -xe

docker build \
	-f distro/docker/Dockerfile \
	-e TRAVIS_PULL_REQUEST="$TRAVIS_PULL_REQUEST" \
	-e encrypted_444f3458e047_key="$encrypted_444f3458e047_key" \
	-e encrypted_444f3458e047_iv="$encrypted_444f3458e047_iv" \
	.