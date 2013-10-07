for obj in om.objects.values():
    if isinstance(obj, om.AffordanceItem):
        obj.publish()
