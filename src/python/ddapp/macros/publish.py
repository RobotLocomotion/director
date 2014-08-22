for obj in om.getObjects():
    if isinstance(obj, vis.AffordanceItem):
        obj.publish()
