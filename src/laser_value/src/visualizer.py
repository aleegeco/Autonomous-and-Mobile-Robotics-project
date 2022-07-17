import cv2

def visibility(img, centre):
    image = img.copy()
    ############################
    # right side
    # sector 1
    y_init = 1
    y = y_init
    x = 0
    while True:
        while True:
            if image[centre[1] - y, centre[0] + x] > 0:
                image[centre[1] - y, centre[0] + x] = 80
            else:
                break
            x += 1
            y += 1
        x = 0
        y_init += 1
        y = y_init
        if image[centre[1] - y, centre[0] + x] == 0:
            break

    # sector 2
    x_init = 0
    x = x_init
    y = 0
    while True:
        while True:
            if image[centre[1] - y, centre[0] + x] > 0:
                image[centre[1] - y, centre[0] + x] = 80
            else:
                break
            x += 1
            y += 1
        y = 0
        x_init += 1
        x = x_init
        if image[centre[1] - y, centre[0] + x] == 0:
            break

    # sector 3
    y_init = 1
    y = y_init
    x = 0
    while True:
        while True:
            if image[centre[1] + y, centre[0] + x] > 0:
                image[centre[1] + y, centre[0] + x] = 80
            else:
                break
            x += 1
            y += 1
        x = 0
        y_init += 1
        y = y_init
        if image[centre[1] + y, centre[0] + x] == 0:
            break
    # sector 4
    x_init = 0
    x = x_init
    y = 0
    while True:
        while True:
            if image[centre[1] + y, centre[0] + x] > 0:
                image[centre[1] + y, centre[0] + x] = 80
            else:
                break
            x += 1
            y += 1
        y = 0
        x_init += 1
        x = x_init
        if image[centre[1] + y, centre[0] + x] == 0:
            break
    ####################################################
    # left side
    # sector 5
    y_init = 1
    y = y_init
    x = 0
    while True:
        while True:
            if image[centre[1] + y, centre[0] - x] > 0:
                image[centre[1] + y, centre[0] - x] = 80
            else:
                break
            x += 1
            y += 1
        x = 0
        y_init += 1
        y = y_init
        if image[centre[1] + y, centre[0] - x] == 0:
            break

    # sector 6
    x_init = 0
    x = x_init
    y = 0
    while True:
        while True:
            if image[centre[1] + y, centre[0] - x] > 0:
                image[centre[1] + y, centre[0] - x] = 80
            else:
                break
            x += 1
            y += 1
        y = 0
        x_init += 1
        x = x_init
        if image[centre[1] + y, centre[0] - x] == 0:
            break

    # sector 7
    y_init = 1
    y = y_init
    x = 0
    while True:
        while True:
            if image[centre[1] - y, centre[0] - x] > 0:
                image[centre[1] - y, centre[0] - x] = 80
            else:
                break
            x += 1
            y += 1
        x = 0
        y_init += 1
        y = y_init
        if image[centre[1] - y, centre[0] - x] == 0:
            break

    # sector 8
    x_init = 0
    x = x_init
    y = 0
    while True:
        while True:
            if image[centre[1] - y, centre[0] - x] > 0:
                image[centre[1] - y, centre[0] - x] = 80
            else:
                break
            x += 1
            y += 1
        y = 0
        x_init += 1
        x = x_init
        if image[centre[1] - y, centre[0] - x] == 0:
            break

    image[centre[1], centre[0]] = 0
    return image