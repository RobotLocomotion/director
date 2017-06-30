#!/bin/bash

set -e

if [ -z "$encrypted_bintray_api_key" ]; then
  echo "encrypted bintray api key not available. ignoring request to upload files:" $*
  exit 0
fi

scriptDir=$(cd $(dirname $0) && pwd)
cd $scriptDir
version=$(git describe)


for filename in "$@"
do
  echo "uploading file: $filename"
  curl -T $filename -u patmarion:$encrypted_bintray_api_key https://api.bintray.com/content/patmarion/director/director/${version}/$(basename $filename)
done
