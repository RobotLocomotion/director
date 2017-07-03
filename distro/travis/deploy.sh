#!/bin/bash
set -ex

cd $(dirname $0)

branch_name=$(cd "$root_dir" && git rev-parse --abbrev-ref HEAD)
repo_path=$(git rev-parse --show-toplevel)
source_dir=$(basename $repo_path)

cd /tmp
rm -rf $source_dir

git clone "$repo_path" -b $branch_name
cd $source_dir/distro/travis

git remote rm origin
git remote add origin https://github.com/robotlocomotion/director.git
git gc

if [ "$branch_name" = "master" ]; then
 tag_name=robotlocomotion/director:latest
else
 tag_name=robotlocomotion/director:$(git describe)
fi

\
  MAKE_PACKAGE=ON \
  USE_LCM=ON \
  USE_LIBBOT=ON \
  DOCKER_DEPLOY=true \
  DOCKER_TAG_NAME=${tag_name} \
  ./test_docker.sh
