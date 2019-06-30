#!/bin/bash

set -e

scriptDir=$(cd $(dirname $0) && pwd)

if [ -z "$encrypted_copyfiles_password" ]; then
  echo "encrypted ssh keys not available. ignoring request to copy files:" $*
  exit 0
fi

openssl aes-256-cbc -k $encrypted_copyfiles_password -in $scriptDir/director_upload_key.enc -out $scriptDir/director_upload_key -d
chmod 600 $scriptDir/director_upload_key

dest=$encrypted_copyfiles_host
user=pat

mkdir -p ~/.ssh
ssh-keyscan -H $dest >> ~/.ssh/known_hosts
scp -i $scriptDir/director_upload_key $* $user@$dest:director_upload_files/
