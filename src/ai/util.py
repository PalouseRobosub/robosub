def getNMostProbable(detection_array, n, thresh=0.1):
    sorted_list = sorted(detection_array, key=lambda x: x.probability, reverse=True)
    return sorted_list[:n]

def getMostProbable(detection_array, thresh=0.1):
    return getNMostProbable(detection_array, 1, thresh)[0]

def normalize(detection_array):
    if type(detection_array) is list:
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
