for obj in om.objects.values():
    if isinstance(obj, vis.AffordanceItem):
        obj.publish()
