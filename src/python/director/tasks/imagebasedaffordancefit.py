from director import cameraview
from director.pointpicker import ImagePointPicker
from director import segmentation

class ImageBasedAffordanceFit(object):

    def __init__(self, imageView=None, numberOfPoints=1):

        self.imageView = imageView or cameraview.CameraImageView(cameraview.imageManager, self.getImageChannel(), 'image view')
        self.imagePicker = ImagePointPicker(self.imageView, numberOfPoints=numberOfPoints)
        self.imagePicker.connectDoubleClickEvent(self.onImageViewDoubleClick)
        self.imagePicker.annotationFunc = self.onImageAnnotation
        self.imagePicker.start()

        self.pointCloudSource = 'lidar'
        self.pickLineRadius = 0.05
        self.pickNearestToCamera = True

    def getImageChannel(self):
        return 'CAMERA_LEFT'

    def getPointCloud(self):
        assert self.pointCloudSource in ('lidar', 'stereo')
        if self.pointCloudSource == 'stereo':
            return segmentation.getDisparityPointCloud(decimation=1, removeOutliers=False)
        else:
            return segmentation.getCurrentRevolutionData()

    def onImageAnnotation(self, *points):
        polyData = self.getPointCloud()
        points = [self.getPointCloudLocationFromImage(p, self.imageView, polyData) for p in points]
        self.fit(polyData, points)

    def getPointCloudLocationFromImage(self, imagePixel, imageView, polyData):
        cameraPos, ray = imageView.getWorldPositionAndRay(imagePixel)
        return segmentation.extractPointsAlongClickRay(cameraPos, ray, polyData, distanceToLineThreshold=self.pickLineRadius, nearestToCamera=self.pickNearestToCamera)

    def onImageViewDoubleClick(self, displayPoint, modifiers, imageView):
        pass

    def fit(self, pointData, points):
        pass
