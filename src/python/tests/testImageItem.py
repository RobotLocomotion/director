from director import imageview
from director import consoleapp
from director import visualization as vis
import director.vtkAll as vtk
import director.vtkNumpy as vnp


def getNewImage():
    # return a vtkImageData object
    s = vtk.vtkRTAnalyticSource()
    s.SetWholeExtent(-100, 100, -100, 100, 0, 0)
    s.Update()
    return s.GetOutput()


app = consoleapp.ConsoleApp()
view = app.createView()

img = getNewImage()
obj = vis.showImage(img, 'test image')
obj.setProperty('Anchor', 'Top Left')
obj.setProperty('Width', 200)

assert obj.image is img

img2 = getNewImage()
vis.updateImage(img2, 'test image')

assert obj.image is img2

view.show()
app.start()
