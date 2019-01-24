import numpy as np
import cv2


def make_line_coordinates(img, line_params):
    # no line detected error catching
    if line_params is None or line_params is np.float64 or line_params.size != 2:
        return np.array([0, 0, 0, 0])

    slope, intercept = line_params
    y1 = img.shape[0]
    y2 = int(y1 * (3.0/5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


def average_slope_intercepts(img, line_data):
    left_fit = []
    right_fit = []
    for line in line_data:
        x1, y1, x2, y2 = line.reshape(4)
        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = make_line_coordinates(img, left_fit_average)
    right_line = make_line_coordinates(img, right_fit_average)
    return np.array([left_line, right_line])


def grey_and_blur(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    return cv2.GaussianBlur(gray, (5, 5), 0)  # 5x5 kernel for blur, 0 deviation


def canny(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # optional since Canny method already does this with 5x5 kernel?
    # then we filter out noise, to stop false edge-positives
    blur = cv2.GaussianBlur(gray, (5, 5), 0)  # 5x5 kernel for blur, 0 deviation

    return cv2.Canny(blur, 50, 150)


def display_lines(img, line_data):
    output_img = np.zeros_like(img)
    if line_data is not None:
        for x1, y1, x2, y2 in line_data:
            cv2.line(output_img, (x1, y1), (x2, y2), (255, 0, 0), 10)

    return output_img


def region_of_interest(img):
    height = img.shape[0]
    polygons = np.array([
        [(200, height), (1100, height), (550, 250)]
    ])

    mask = np.zeros_like(img)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(img, mask)

