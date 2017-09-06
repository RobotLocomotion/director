from director import imageview
from director import consoleapp
import director.vtkAll as vtk
import director.vtkNumpy as vnp


# create a vtkImageData object
s = vtk.vtkRTAnalyticSource()
s.SetWholeExtent(-100, 100, -100, 100, 0, 0)
s.Update()
image = s.GetOutput()

# get image data as a numpy array
w,h,_ = image.GetDimensions()
img = vnp.getNumpyFromVtk(image, 'RTData').reshape(h,w,-1)

# show numpy image data
view = imageview.ImageView()
view.showNumpyImage(img)
view.show()

# convert back to vtkImageData and show
view2 = imageview.ImageView()
view2.setImage(vnp.numpyToImageData(img))
view2.show()

consoleapp.ConsoleApp.start()
