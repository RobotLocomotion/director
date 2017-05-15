def deepCopy(dataObj):
    newData = dataObj.NewInstance()
    newData.DeepCopy(dataObj)
    return newData

def shallowCopy(dataObj):
    newData = dataObj.NewInstance()
    newData.ShallowCopy(dataObj)
    return newData
