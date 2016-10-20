#!/bin/bash

scriptDir=$(cd $(dirname $0) && pwd)

openssl aes-256-cbc -K $encrypted_444f3458e047_key -iv $encrypted_444f3458e047_iv -in $scriptDir/travisci.key.enc -out $scriptDir/travisci.key -d
chmod 600 $scriptDir/travisci.key

dest=$encrypted_copyfiles_host
user=travis

ssh-keyscan -H $dest >> ~/.ssh/known_hosts
scp -i $scriptDir/travisci.key $* $user@$dest:files/
