def getNMostProbable(detection_array, n, thresh=0.1):
    if detection_array is None:
        return None

    sorted_list = sorted(detection_array, key=lambda x: x.probability,
                         reverse=True)
    return sorted_list[:n]

def getMostProbable(detection_array, thresh=0.1):
    results = getNMostProbable(detection_array, 1, thresh)
    return results[0] if len(results) != 0 else None

def normalize(detection_array):
    if detection_array is None:
        return
    elif type(detection_array) is list:
        for detection in detection_array:
            detection.x = (detection.x - 0.5) * 2
            detection.y = (detection.y - 0.5) * 2
    else:
        detection_array.x = (detection_array.x - 0.5) * 2
        detection_array.y = (detection_array.y - 0.5) * 2

def filterByLabel(detection_array, label_name, thresh=0.1):
    detections = []

    for detection in detection_array:
        if label_name in detection.label and detection > thresh:
            detections.append(detection)

    return detections

def wrap_yaw(yaw):
    while yaw >= 180:
        yaw = yaw - 360

    while yaw < -180:
        yaw = yaw + 360

    return yaw
