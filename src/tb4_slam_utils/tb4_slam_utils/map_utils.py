def analyze_map(map_msg):
    data = map_msg.data

    free = sum(1 for v in data if v == 0)
    occupied = sum(1 for v in data if v > 0)
    unknown = sum(1 for v in data if v == -1)

    return {
        'width': map_msg.info.width,
        'height': map_msg.info.height,
        'resolution': map_msg.info.resolution,
        'free': free,
        'occupied': occupied,
        'unknown': unknown,
        'total': len(data),
    }