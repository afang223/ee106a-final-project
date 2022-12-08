#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy

# TODO: custom message
from std_msgs.msg import String
from ik.msg import board_state, board_row

# MIN_BLUE_HSV = (100, 20, 20)
# MAX_BLUE_HSV = (260, 255,255)
# CROP = 10
# POLAR_PARAM = 300
# LINE_ANGLE_CUTOFF = 9
# WIDTH_CIRCLES = 7
# HEIGHT_CIRCLES = 6
# MULTIPLIER_LENGTH = 100

# OFFSET = int(0.6 * MULTIPLIER_LENGTH)
# SQUARE_SIZE = int(0.15 * MULTIPLIER_LENGTH)
# CIRCLE_PIXELS = int(0.93 * MULTIPLIER_LENGTH)

# MIN_RED_HSV = [(0, 70, 90), (130, 70, 90)]
# MAX_RED_HSV = [(40, 255, 255), (180, 255, 255)]

# MIN_YELLOW_HSV = [(40, 150, 150)]
# MAX_YELLOW_HSV = [(70, 255, 255)]

# YELLOW = -1
# NONE = 0
# RED = 1

# LAST_BOARD_MOVE = None#np.zeros((HEIGHT_CIRCLES, WIDTH_CIRCLES))
# LAST_BOARD = None#np.zeros((HEIGHT_CIRCLES, WIDTH_CIRCLES))
# CURR_BOARD = np.zeros((HEIGHT_CIRCLES, WIDTH_CIRCLES))
# NUM_DUPLICATES = 0

# DUPLICATE_CUTOFF = 3

# INTERSECT_PTS_LAST = None
# OUTPUT_ANIMATION = np.zeros((HEIGHT_CIRCLES*MULTIPLIER_LENGTH + MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH*2, 3))

def cyclic_intersection_pts(pts):
    """
    Sorts 4 points in clockwise direction with the first point been closest to 0,0
    Assumption:
        There are exactly 4 points in the input and
        from a rectangle which is not very distorted
    """
    if pts.shape[0] != 4:
        return None

    # Calculate the center
    center = np.mean(pts, axis=0)

    # Sort the points in clockwise
    cyclic_pts = [
        # Top-left
        pts[np.where(np.logical_and(pts[:, 0] < center[0], pts[:, 1] < center[1]))[0][0], :],
        # Top-right
        pts[np.where(np.logical_and(pts[:, 0] > center[0], pts[:, 1] < center[1]))[0][0], :],
        # Bottom-Right
        pts[np.where(np.logical_and(pts[:, 0] > center[0], pts[:, 1] > center[1]))[0][0], :],
        # Bottom-Left
        pts[np.where(np.logical_and(pts[:, 0] < center[0], pts[:, 1] > center[1]))[0][0], :]
    ]

    return np.array(cyclic_pts)
def polar2cartesian(rho: float, theta_rad: float, rotate90: bool = False):
    """
    Converts line equation from polar to cartesian coordinates
    Args:
        rho: input line rho
        theta_rad: input line theta
        rotate90: output line perpendicular to the input line
    Returns:
        m: slope of the line
           For horizontal line: m = 0
           For vertical line: m = np.nan
        b: intercept when x=0
    """
    x = np.cos(theta_rad) * rho
    y = np.sin(theta_rad) * rho
    m = np.nan
    if not np.isclose(x, 0.0):
        m = y / x
    if rotate90:
        if m is np.nan:
            m = 0.0
        elif np.isclose(m, 0.0):
            m = np.nan
        else:
            m = -1.0 / m
    b = 0.0
    if m is not np.nan:
        b = y - m * x

    return m, b
def solve4x(y: float, m: float, b: float):
    """
    From y = m * x + b
         x = (y - b) / m
    """
    if np.isclose(m, 0.0):
        return 0.0
    if m is np.nan:
        return b
    return (y - b) / m
def solve4y(x: float, m: float, b: float):
    """
    y = m * x + b
    """
    if m is np.nan:
        return b
    return m * x + b
def intersection(m1: float, b1: float, m2: float, b2: float):
    # Consider y to be equal and solve for x
    # Solve:
    #   m1 * x + b1 = m2 * x + b2
    if b2 is np.nan or b1 is np.nan or m1 is np.nan or m2 is np.nan or m1 - m2 == 0:# 0.0000000001:
        return None, None
    x = (b2 - b1) / (m1 - m2)
    # Use the value of x to calculate y
    y = m1 * x + b1

    return int(round(x)), int(round(y))
def line_end_points_on_image(rho: float, theta: float, image_shape: tuple):
    """
    Returns end points of the line on the end of the image
    Args:
        rho: input line rho
        theta: input line theta
        image_shape: shape of the image
    Returns:
        list: [(x1, y1), (x2, y2)]
    """
    m, b = polar2cartesian(rho, theta, True)

    end_pts = []

    if not np.isclose(m, 0.0):
        x = int(0)
        y = int(solve4y(x, m, b))
        if point_on_image(x, y, image_shape):
            end_pts.append((x, y))
            x = int(image_shape[1] - 1)
            y = int(solve4y(x, m, b))
            if point_on_image(x, y, image_shape):
                end_pts.append((x, y))

    if m is not np.nan:
        y = int(0)
        x = int(solve4x(y, m, b))
        if point_on_image(x, y, image_shape):
            end_pts.append((x, y))
            y = int(image_shape[0] - 1)
            x = int(solve4x(y, m, b))
            if point_on_image(x, y, image_shape):
                end_pts.append((x, y))

    return end_pts
def hough_lines_end_points(lines: np.array, image_shape: tuple):
    """
    Returns end points of the lines on the edge of the image
    """
    if len(lines.shape) == 3 and \
            lines.shape[1] == 1 and lines.shape[2] == 2:
        lines = np.squeeze(lines)
    end_pts = []
    for line in lines:
        rho, theta = line
        end_pts.append(
            line_end_points_on_image(rho, theta, image_shape))
    return end_pts
def hough_lines_intersection(lines: np.array, image_shape: tuple):
    """
    Returns the intersection points that lie on the image
    for all combinations of the lines
    """
    if len(lines.shape) == 3 and \
            lines.shape[1] == 1 and lines.shape[2] == 2:
        lines = np.squeeze(lines)
    lines_count = len(lines)
    intersect_pts = []
    for i in range(lines_count - 1):
        for j in range(i + 1, lines_count):
            m1, b1 = polar2cartesian(lines[i][0], lines[i][1], True)
            m2, b2 = polar2cartesian(lines[j][0], lines[j][1], True)
            x, y = intersection(m1, b1, m2, b2)
            # print(x, y)
            if x is not None and y is not None and point_on_image(x, y, image_shape):
                intersect_pts.append([x, y])
                # print("appended")
    return np.array(intersect_pts, dtype=int)
def drawHoughLines(image, lines, output):
    out = image.copy()
    for line in lines:
        rho, theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 10000 * (-b))
        y1 = int(y0 + 10000 * (a))
        x2 = int(x0 - 10000 * (-b))
        y2 = int(y0 - 10000 * (a))
        cv2.line(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.imwrite(output, out)
def point_on_image(x: int, y: int, image_shape: tuple):
    """
    Returns true is x and y are on the image
    """
    return 0 <= y < image_shape[0] and 0 <= x < image_shape[1]

def classify_token_color(patch):
    # Path size: N x N x 3
    avg_color = np.mean(patch, axis=(0, 1)) # (3)
    avg_color = np.uint8(avg_color.reshape((1, 1, 3)))#.astype(np.int32)
    # print(avg_color)
    # print(image.shape)
    hsv = cv2.cvtColor(avg_color, cv2.COLOR_BGR2HSV)
    # print(hsv)
    yellow = False
    for i in range(len(MIN_YELLOW_HSV)):
        yellow = yellow or cv2.inRange(hsv, MIN_YELLOW_HSV[i], MAX_YELLOW_HSV[i])[0]
    red = False
    for i in range(len(MIN_RED_HSV)):
        red = red or cv2.inRange(hsv, MIN_RED_HSV[i], MAX_RED_HSV[i])[0]
    if yellow:
        return YELLOW
    elif red:
        return RED
    return NONE


def video_cap():
    global MIN_BLUE_HSV
    global MAX_BLUE_HSV
    global CROP
    global POLAR_PARAM
    global LINE_ANGLE_CUTOFF
    global WIDTH_CIRCLES
    global HEIGHT_CIRCLES
    global MULTIPLIER_LENGTH

    global OFFSET
    global SQUARE_SIZE
    global CIRCLE_PIXELS

    global MIN_RED_HSV
    global MAX_RED_HSV

    global MIN_YELLOW_HSV
    global MAX_YELLOW_HSV

    global YELLOW
    global NONE
    global RED

    global LAST_BOARD_MOVE
    global LAST_BOARD
    global CURR_BOARD
    global NUM_DUPLICATES

    global DUPLICATE_CUTOFF

    global INTERSECT_PTS_LAST
    global OUTPUT_ANIMATION
    pub = rospy.Publisher('board_state', board_state, queue_size=10)


    cap = cv2.VideoCapture(0)#'10000000_5601545573271263_6247416547873940391_n.mp4')#board_background.mp4')
    output = []
    OUTPUT_ANIMATION_vid = []
    print('Press space bar to lock in camera...')
    command = ' '
    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        
        if cv2.waitKey(1) == ord(command):
            if command == 'q':
                break
            print("Press 'q' to exit program...")
            command = 'q'
        if command == ' ':
            cv2.imshow('frame', frame) 
            continue
            # break
    # print('done')      
    # cap.release()
    # print('released')
    # cv2.destroyAllWindows()
    # print('destroy')

    # cap = cv2.VideoCapture(0)       
    # print('new capture')  
    # while cap.isOpened():
    #     print('reading')
    #     ret, frame = cap.read()
    #     print('has read')
        # if not ret:
        #     break
        image = frame
        # image = frame
        # print(image.dtype)
        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Mask the blue color of the board
            mask = cv2.inRange(hsv, MIN_BLUE_HSV, MAX_BLUE_HSV)
            zeros = np.ones_like(image) * 255
            # convert board to white and everything else to black
            target = cv2.bitwise_and(zeros,zeros, mask=mask) 
            gray = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
            # cv2.imwrite("mask.png", gray)
            # find contours
            contours, hierarchy = cv2.findContours(gray,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            largest_contour_idx = np.argmax([cv2.contourArea(c) for c in contours])
            cv2.drawContours(image, contours, largest_contour_idx, (255,255,255), 3)
            # cv2.imwrite("contours.png", image)
            blank = np.zeros_like(image)
            cv2.drawContours(blank, contours, largest_contour_idx, (255,255,255), 3)
            blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)

            blank = blank[CROP:-CROP, CROP:-CROP]
            image = image[CROP:-CROP, CROP:-CROP]
            polar_lines = cv2.HoughLines(blank, 1, np.pi / 180, POLAR_PARAM, min_theta=1e-9)#, max_theta=np.pi - 1e-9)
            polar_lines_squeeze = np.squeeze(polar_lines)

            polar_lines = polar_lines[(polar_lines_squeeze[:, 1] < np.pi/LINE_ANGLE_CUTOFF) | (polar_lines_squeeze[:, 1] > (LINE_ANGLE_CUTOFF-1)*np.pi/LINE_ANGLE_CUTOFF) | ((polar_lines_squeeze[:, 1] > (LINE_ANGLE_CUTOFF-1)*np.pi/LINE_ANGLE_CUTOFF/2) &  (polar_lines_squeeze[:, 1] < (LINE_ANGLE_CUTOFF+1)*np.pi/LINE_ANGLE_CUTOFF/2))]
            polar_lines[polar_lines[:, 0, 1] > (LINE_ANGLE_CUTOFF-1)*np.pi/LINE_ANGLE_CUTOFF, 0, 0] *= -1
            # print(polar_lines[polar_lines[:, 0, 1] <= np.pi/LINE_ANGLE_CUTOFF][:, 0, 1])
            polar_lines[polar_lines[:, 0, 1] > (LINE_ANGLE_CUTOFF-1)*np.pi/LINE_ANGLE_CUTOFF, 0, 1] -= np.pi
            # print(polar_lines[polar_lines[:, 0, 1] <= np.pi/LINE_ANGLE_CUTOFF][:, 0, 1])
            # print(polar_lines[polar_lines[:, 0, 1] > np.pi][:, 0, 1])
            # drawHoughLines(image, polar_lines, 'houghlines.png')
            # plt.plot(polar_lines[:, 0, 0], polar_lines[:, 0, 1], 'o', color='black');
            # plt.savefig('clusters.png')
            from sklearn.cluster import KMeans
            kmeans = KMeans(n_clusters=4, n_init=10, max_iter=300).fit(polar_lines[:, 0, :])
            # print(kmeans.cluster_centers_)
            # print(np.expand_dims(kmeans.cluster_centers_, axis=0).shape)
            # print(np.expand_dims(kmeans.cluster_centers_, axis=1).shape)
            # print(np.expand_dims(kmeans.cluster_centers_, axis=2).shape)
            polar_lines =  np.expand_dims(kmeans.cluster_centers_, axis=1)

            # drawHoughLines(image, polar_lines, 'houghlines2.png')

            # Detect the intersection points
            intersect_pts = hough_lines_intersection(polar_lines, blank.shape)
            # Sort the points in cyclic order
            intersect_pts = cyclic_intersection_pts(intersect_pts)
            if intersect_pts is None:
                raise Exception("Intersections not found")  
            INTERSECT_PTS_LAST = intersect_pts
            # for pt in intersect_pts:
            #     cv2.circle(image, (pt[0], pt[1]), radius=5, thickness=-1, color=(0, 255, 0))
            # cv2.imwrite('intersections.png',image)
        except:
            print("Homography failed...")
            intersect_pts = INTERSECT_PTS_LAST
            if intersect_pts is None:
                continue
        h, status = cv2.findHomography(intersect_pts, np.array([[0, 0], [WIDTH_CIRCLES*MULTIPLIER_LENGTH, 0], [WIDTH_CIRCLES*MULTIPLIER_LENGTH, HEIGHT_CIRCLES*MULTIPLIER_LENGTH], [0, HEIGHT_CIRCLES*MULTIPLIER_LENGTH]]))
        im_dst = cv2.warpPerspective(image, h, (WIDTH_CIRCLES*MULTIPLIER_LENGTH,  HEIGHT_CIRCLES*MULTIPLIER_LENGTH))

        # cv2.imshow('image',im_dst)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

        
        for i in range(WIDTH_CIRCLES):
            for j in range(HEIGHT_CIRCLES):
                # cv2.circle(output[0], (65 + i * 93, 65 + 93 * j), radius=5, thickness=-1, color=(0, 255, 0))
                # cv2.circle(output[0], (70 + i * 93, 70 + 93 * j), radius=5, thickness=-1, color=(0, 255, 0))
                min_x = OFFSET + i * CIRCLE_PIXELS
                min_y = OFFSET + j * CIRCLE_PIXELS
                max_x = min_x + SQUARE_SIZE
                max_y = min_y + SQUARE_SIZE
                cv2.rectangle(im_dst, (min_x, min_y), (max_x, max_y), color=(0, 255, 0), thickness=2)
                patch = im_dst[min_y:max_y, min_x:max_x]
                # print(patch.shape)
                token_type = classify_token_color(patch)
                CURR_BOARD[j, i] = token_type
                text = "none" if token_type == NONE else "red" if token_type == RED else "yellow"
                
                # Using cv2.putText() method
                im_dst = cv2.putText(im_dst, text, (max_x, max_y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                        fontScale=0.3, color=(255, 0, 0), thickness=1)
        # cv2.imwrite('homo.png',im_dst)
        # print(im_dst)
        # OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, 0:WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = np.uint8(im_dst.copy())
        
        if LAST_BOARD is None or (LAST_BOARD == CURR_BOARD).all():
            NUM_DUPLICATES += 1
            # print(LAST_BOARD, CURR_BOARD)
            LAST_BOARD = CURR_BOARD.copy()
        else:
            LAST_BOARD = CURR_BOARD.copy()
            NUM_DUPLICATES = 0
        # print(CURR_BOARD)
        print(NUM_DUPLICATES)
        if NUM_DUPLICATES == DUPLICATE_CUTOFF:
            # check if next move is valid
            if np.sum(np.abs(CURR_BOARD)) == 0:
                print("Board cleared")
                LAST_BOARD_MOVE = CURR_BOARD.copy()
                OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = (255, 0, 0)
                OUTPUT_ANIMATION[HEIGHT_CIRCLES*MULTIPLIER_LENGTH:HEIGHT_CIRCLES*MULTIPLIER_LENGTH + MULTIPLIER_LENGTH, 0:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = (0, 0, 0)
                for i in range(WIDTH_CIRCLES):
                    for j in range(HEIGHT_CIRCLES):
                        min_x = OFFSET + i * CIRCLE_PIXELS
                        min_y = OFFSET + j * CIRCLE_PIXELS
                        color = (0, 0, 255) if CURR_BOARD[j, i] == RED else (255, 255, 0) if CURR_BOARD[j, i] == YELLOW else (0, 0, 0)
                        cv2.circle(OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :], (min_x, min_y), color=color, radius=30, thickness=-1)
                OUTPUT_ANIMATION = cv2.putText(OUTPUT_ANIMATION, "Last Move: Board Cleared", (300, 50 + HEIGHT_CIRCLES*MULTIPLIER_LENGTH), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                        fontScale=1, color=(255, 255, 255), thickness=2)
                pub_msg = board_state()
                pub_msg.player = 'NONE'
                pub_msg.column = int(-1)
                pub_msg.board = [board_row(list(row)) for row in CURR_BOARD.astype(np.int8)]
                pub.publish(pub_msg)
            elif LAST_BOARD_MOVE is None:
                print("new initialization")
                print(CURR_BOARD)
                LAST_BOARD_MOVE = CURR_BOARD.copy()
                OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = (255, 0, 0)
                OUTPUT_ANIMATION[HEIGHT_CIRCLES*MULTIPLIER_LENGTH:HEIGHT_CIRCLES*MULTIPLIER_LENGTH + MULTIPLIER_LENGTH, 0:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = (0, 0, 0)
                OUTPUT_ANIMATION = cv2.putText(OUTPUT_ANIMATION, "Last Move: New Initialization", (300, 50 + HEIGHT_CIRCLES*MULTIPLIER_LENGTH), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                        fontScale=1, color=(255, 255, 255), thickness=2)
                for i in range(WIDTH_CIRCLES):
                    for j in range(HEIGHT_CIRCLES):
                        min_x = OFFSET + i * CIRCLE_PIXELS
                        min_y = OFFSET + j * CIRCLE_PIXELS
                        color = (0, 0, 255) if CURR_BOARD[j, i] == RED else (255, 255, 0) if CURR_BOARD[j, i] == YELLOW else (0, 0, 0)
                        cv2.circle(OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :], (min_x, min_y), color=color, radius=30, thickness=-1)
                pub_msg = board_state()
                pub_msg.player = 'NONE'
                pub_msg.column = int(-1)
                pub_msg.board = [board_row(list(row)) for row in CURR_BOARD.astype(np.int8)]
                pub.publish(pub_msg)
            elif np.sum(np.abs(LAST_BOARD_MOVE - CURR_BOARD)) == 1:
                print("1 Token spot changed")
                bool_mask = np.abs(LAST_BOARD_MOVE - CURR_BOARD) == 1
                where = np.where(bool_mask)
                where_y, where_x = where[0][0], where[1][0]
                token_added = CURR_BOARD[bool_mask][0]
                if token_added == NONE:
                    print("token changed to none???...")
                else:
                    if token_added == YELLOW:
                        print(f"Yellow added token in column {where_x}")
                    else:
                        print(f"Red added token in column {where_x}")
                    if (where_y == HEIGHT_CIRCLES - 1 or CURR_BOARD[where_y + 1, where_x] != NONE) and (where_y == 0 or CURR_BOARD[where_y - 1, where_x] == NONE):
                        print("new valid state found")
                        print(CURR_BOARD)
                        LAST_BOARD_MOVE = CURR_BOARD.copy()
                        OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = (255, 0, 0)
                        OUTPUT_ANIMATION[HEIGHT_CIRCLES*MULTIPLIER_LENGTH:HEIGHT_CIRCLES*MULTIPLIER_LENGTH + MULTIPLIER_LENGTH, 0:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = (0, 0, 0)
                        pub_string = f"Last Move: Added {'YELLOW' if token_added == YELLOW else 'RED'} token to column {where_x}!"
                        OUTPUT_ANIMATION = cv2.putText(OUTPUT_ANIMATION, pub_string, (300, 50 + HEIGHT_CIRCLES*MULTIPLIER_LENGTH), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)
                        pub_msg = board_state()
                        pub_msg.player = 'YELLOW' if token_added == YELLOW else 'RED'
                        pub_msg.column = int(where_x)
                        pub_msg.board = [board_row(list(row)) for row in CURR_BOARD.astype(np.int8)]
                        pub.publish(pub_msg)
                        print(rospy.get_name() + ": I sent \"%s\"" % pub_string)
        
                        for i in range(WIDTH_CIRCLES):
                            for j in range(HEIGHT_CIRCLES):
                                min_x = OFFSET + i * CIRCLE_PIXELS
                                min_y = OFFSET + j * CIRCLE_PIXELS
                                color = (0, 0, 255) if CURR_BOARD[j, i] == RED else (0, 255, 255) if CURR_BOARD[j, i] == YELLOW else (0, 0, 0)
                                cv2.circle(OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH:2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, :], (min_x, min_y), color=color, radius=30, thickness=-1)
        OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, 0:WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = np.float64(im_dst/255.0)
        # print(OUTPUT_ANIMATION.dtype)
        # print(im_dst.dtype)
        cv2.imshow('frame', OUTPUT_ANIMATION.copy())
        OUTPUT_ANIMATION[0:HEIGHT_CIRCLES*MULTIPLIER_LENGTH, 0:WIDTH_CIRCLES*MULTIPLIER_LENGTH, :] = im_dst
        # OUTPUT_ANIMATION_vid.append(OUTPUT_ANIMATION.copy()) 
        # output.append(im_dst)
        if cv2.waitKey(1) == ord('q'):
            break                      

    cap.release()
    cv2.destroyAllWindows()

    # output_video = cv2.VideoWriter('board_homography_labels2.mp4',cv2.VideoWriter_fourcc(*'MP4V'),30.0,(WIDTH_CIRCLES*MULTIPLIER_LENGTH, HEIGHT_CIRCLES*MULTIPLIER_LENGTH))
    # for j in range(len(output)):
    #     output_video.write(output[j])
    #     output_video.release()
    #     output_video = cv2.VideoWriter('board_homography_labels_animate2.mp4',cv2.VideoWriter_fourcc(*'MP4V'),30.0,(2*WIDTH_CIRCLES*MULTIPLIER_LENGTH, HEIGHT_CIRCLES*MULTIPLIER_LENGTH+MULTIPLIER_LENGTH))
    # for j in range(len(OUTPUT_ANIMATION_vid)):
    #     output_video.write(np.uint8(OUTPUT_ANIMATION_vid[j]))
    #     output_video.release()

if __name__ == '__main__':

    # global MIN_BLUE_HSV
    # global MAX_BLUE_HSV
    # global CROP
    # global POLAR_PARAM
    # global LINE_ANGLE_CUTOFF
    # global WIDTH_CIRCLES
    # global HEIGHT_CIRCLES
    # global MULTIPLIER_LENGTH

    # global OFFSET
    # global SQUARE_SIZE
    # global CIRCLE_PIXELS

    # global MIN_RED_HSV
    # global MAX_RED_HSV

    # global MIN_YELLOW_HSV
    # global MAX_YELLOW_HSV

    # global YELLOW
    # global NONE
    # global REDDUPLICATE_CUTOFF

    # global LAST_BOARD_MOVE
    # global LAST_BOARD
    # global CURR_BOARD
    # global NUM_DUPLICATES

    # global DUPLICATE_CUTOFF

    # global INTERSECT_PTS_LAST
    # global OUTPUT_ANIMATION

    MIN_BLUE_HSV = (100, 20, 20)
    MAX_BLUE_HSV = (260, 255,255)
    CROP = 10
    POLAR_PARAM = 300
    LINE_ANGLE_CUTOFF = 9
    WIDTH_CIRCLES = 7
    HEIGHT_CIRCLES = 6
    MULTIPLIER_LENGTH = 100

    OFFSET = int(0.6 * MULTIPLIER_LENGTH)
    SQUARE_SIZE = int(0.15 * MULTIPLIER_LENGTH)
    CIRCLE_PIXELS = int(0.93 * MULTIPLIER_LENGTH)

    MIN_RED_HSV = [(0, 70, 90), (130, 70, 90)]
    MAX_RED_HSV = [(40, 255, 255), (180, 255, 255)]

    MIN_YELLOW_HSV = [(40, 150, 150)]
    MAX_YELLOW_HSV = [(70, 255, 255)]

    YELLOW = -1
    NONE = 0
    RED = 1

    LAST_BOARD_MOVE = None#np.zeros((HEIGHT_CIRCLES, WIDTH_CIRCLES))
    LAST_BOARD = None#np.zeros((HEIGHT_CIRCLES, WIDTH_CIRCLES))
    CURR_BOARD = np.zeros((HEIGHT_CIRCLES, WIDTH_CIRCLES))
    NUM_DUPLICATES = 0

    DUPLICATE_CUTOFF = 3

    INTERSECT_PTS_LAST = None
    OUTPUT_ANIMATION = np.zeros((HEIGHT_CIRCLES*MULTIPLIER_LENGTH + MULTIPLIER_LENGTH, WIDTH_CIRCLES*MULTIPLIER_LENGTH*2, 3))
    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('vision', anonymous=True)

    # Check if the notalkerde has received a signal to shut down. If not, run the
    # talker method.
    try:
        video_cap()
    except rospy.ROSInterruptException: pass

