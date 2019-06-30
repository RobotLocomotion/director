FROM ubuntu:14.04

ARG USE_LCM
ARG USE_LCMGL
ARG USE_LIBBOT
ARG MAKE_DOCS
ARG MAKE_PACKAGE
ARG DOCKER_DEPLOY
ARG TRAVIS_OS_NAME
ARG TRAVIS_BRANCH
ARG TRAVIS_PULL_REQUEST
ARG TRAVIS_PULL_REQUEST_BRANCH
ARG TRAVIS_PULL_REQUEST_SHA
ARG TRAVIS_BUILD_DIR
ARG encrypted_444f3458e047_key
ARG encrypted_444f3458e047_iv
ARG encrypted_copyfiles_host
ARG encrypted_copyfiles_password
ARG encrypted_bintray_api_key

ENV \
    DISPLAY=":99.0" \
    USE_LCM="${USE_LCM}" \
    USE_LCMGL="${USE_LCMGL}" \
    USE_LIBBOT="${USE_LIBBOT}" \
    MAKE_DOCS="${MAKE_DOCS}" \
    MAKE_PACKAGE="${MAKE_PACKAGE}" \
    DOCKER_DEPLOY="${DOCKER_DEPLOY}" \
    TRAVIS_OS_NAME="${TRAVIS_OS_NAME}" \
    TRAVIS_BRANCH="${TRAVIS_BRANCH}" \
    TRAVIS_PULL_REQUEST="${TRAVIS_PULL_REQUEST}" \
    TRAVIS_PULL_REQUEST_BRANCH="${TRAVIS_PULL_REQUEST_BRANCH}" \
    TRAVIS_PULL_REQUEST_SHA="${TRAVIS_PULL_REQUEST_SHA}" \
    TRAVIS_BUILD_DIR="${TRAVIS_BUILD_DIR}" \
    encrypted_444f3458e047_key="${encrypted_444f3458e047_key}" \
    encrypted_444f3458e047_iv="${encrypted_444f3458e047_iv}" \
    encrypted_copyfiles_host="${encrypted_copyfiles_host}" \
    encrypted_copyfiles_password="${encrypted_copyfiles_password}" \
    encrypted_bintray_api_key="${encrypted_bintray_api_key}"


COPY . ${TRAVIS_BUILD_DIR}
WORKDIR ${TRAVIS_BUILD_DIR}
RUN ${TRAVIS_BUILD_DIR}/distro/travis/test_main.sh
