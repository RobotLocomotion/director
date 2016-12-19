set -e

docker build \
	-f distro/docker/Dockerfile \
	--build-arg MAKE_PACKAGE=ON \
	--build-arg USE_LCM=ON \
	--build-arg TRAVIS_PULL_REQUEST="$TRAVIS_PULL_REQUEST" \
	--build-arg TRAVIS_OS_NAME="$TRAVIS_OS_NAME" \
	--build-arg encrypted_444f3458e047_key="$encrypted_444f3458e047_key" \
	--build-arg encrypted_444f3458e047_iv="$encrypted_444f3458e047_iv" \
	--build-arg encrypted_copyfiles_host="$encrypted_copyfiles_host" \
	.
