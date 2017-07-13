def getNMostProbable(detection_array, label_name, n, thresh=0.1):
    sorted_list = sorted(detection_array, key=lambda x: x.probability, reverse=True)
    return sorted_list[:n]

def getMostProbable(detection_array, label_name, thresh=0.1):
    return getNMostProbable(detection_array, label_name, n, thresh)

def filterByLabel(detection_array, label_name, thresh=0.1):
    detections = []

    for detection in detection_array:
        if label_name in detection.label and detection > thresh:
            detections.append(detection)

    return detections
