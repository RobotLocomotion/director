docker build -f vtk.dockerfile -t vtk:tmp .


docker run -it vtk:tmp bash -c "scp vtk*.tar.gz pat@patmarion.com:public_html/bottles/"


#id=$(docker create vtk:tmp)
#docker cp -a $id:/root/vtk7.1-qt5.9-python2.7-ubuntu18.04.tar.gz ./
#docker rm -v $id
