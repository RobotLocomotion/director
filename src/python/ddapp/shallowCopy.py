def deepCopy(dataOb):
    newData = dataObject.NewInstance()
    newData.DeepCopy(dataObj)
    return newData

def shallowCopy(dataObj):
    newData = dataObj.NewInstance()
    newData.ShallowCopy(dataObj)
    return newData
