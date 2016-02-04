#!/bin/bash

scriptDir=$(cd $(dirname $0) && pwd)
tar -zxf $scriptDir/test_key.enc -C /tmp
scp -i /tmp/test_key -o StrictHostKeyChecking=no -r $*  pat@128.30.27.15:/tmp/
