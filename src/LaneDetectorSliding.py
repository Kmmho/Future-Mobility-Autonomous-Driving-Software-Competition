# -*- coding: utf-8 -*-
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import cv2
import numpy as np

class LaneDetector():
    ERR_NO_WHITE_PIXELS = 0
    ERR_HISTOGRAM_NOT_VALID = 1
    ERR_NOT_VALID = 2
    ERR_NOT_FOUND = 3
    SUCCESS_POLY_FIT_ONE = -1
    SUCCESS_POLY_FIT_BOTH = -2

    def __init__(self, 
                 img_shape,                 # img size
                 pt_src=None,                    # perspective transformation (src)
                 pt_dst=None,                    # perspective transformation (dst)
                 roi_vertices=None,              # ROI crop coordinate
                 nb_windows = 24,           # number of sliding windows
                 margin = 50,               # width of the windows +/- margin
                 minpix = 20,               # min number of pixels needed to recenter the window
                 min_lane_pts = 400,        # min number of 'hot' pixels needed to fit a 2nd order polynomial as a lane line
                 canny_th1=60, canny_th2=75,
                 discontinue_threshold = 13,
                 lane_width = 354):
        self.img_width, self.img_height = img_shape
        self.half_width = self.img_width // 2
        self.nb_windows = nb_windows
        self.margin = margin
        self.minpix = minpix
        self.min_lane_pts = min_lane_pts
        self.window_height = int(self.img_height / self.nb_windows)
        self.offset = 20
        self.discontinue_threshold = discontinue_threshold
        self.canny_th1, self.canny_th2 = canny_th1, canny_th2
        self.lane_width = lane_width


        self.pt_src = np.float32([(1070, 400),
                                  (230, 400), 
                                  (50, 620), 
                                  (1260, 620)])
        
        self.pt_dst = np.float32([(self.img_width - 150, 0),
                                  (150, 0),
                                  (150, self.img_height),
                                  (self.img_width - 150, self.img_height)])

        self.vertices = np.array([[[100, self.img_height],
                                   [100, 0],
                                   [self.img_width-100, 0],
                                   [self.img_width-100, self.img_height]]], dtype=np.int32)

    def do(self, img, debug = False):
        roi_img = self.preprocess(img)  # 원본 이미지 탑-다운 뷰로 변환하는 전처리 과정        
        binary_img = self.binarization(roi_img) # 탑-다운뷰 이미지 이진화
        out_img = img.copy()
        
        ret, out_polyfit_img, left_fit, right_fit, left_angle, right_angle = self.polyfit(binary_img, debug) # 이진화 이미지에서 차선 인식

        output = [-1,-1, -1, -1, -1]
        if ret <= LaneDetector.SUCCESS_POLY_FIT_ONE:
            # 하나 이상 차선을 찾은 경우
            if left_fit is not None:
                output[3] = left_angle
            if right_fit is not None:
                output[4] = right_angle
                
            # 원본 이미지에 찾은 차선의 2차 곡선 그려 넣기
            out_img, output[0], output[1], output[2] = self.draw(img, roi_img, self.invM, left_fit, right_fit)

        # 두 개의 차선을 찾았는데, 두 차선이 서로 다른 방향을 바라보면 잘못된 정보라고 판단
        if ret==LaneDetector.SUCCESS_POLY_FIT_BOTH and abs(output[3]-output[4]) > 5:
            ret = LaneDetector.ERR_NOT_VALID
            
        return ret, out_img, out_polyfit_img, output
        
    def preprocess(self, img):
        # 1. Perspective transformation
        self.M = cv2.getPerspectiveTransform(self.pt_src, self.pt_dst) # pt_src좌표에서 pt_dst좌표로 대응되는 원근 변환 행렬(탑-다운뷰료 변환)
        self.invM = cv2.getPerspectiveTransform(self.pt_dst, self.pt_src) # pt_dst좌표에서 pt_src좌표로 대응되는 원근 변환 행렬(탑-다운뷰에서 원본 이미지로 변환)
        warped = cv2.warpPerspective(img, self.M, (self.img_width, self.img_height), flags=cv2.INTER_LINEAR) # 원본 이미지를 탑-다운뷰 이미지로 변환
        
        # 2. ROI crop    
        if len(warped.shape) == 3:
            fill_color = (255,) * 3
        else:
            fill_color = 255
                
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)

        # edge_img = cv2.Canny(np.uint8(blur_gray), self.canny_th1, self.canny_th2)
        edge_img = blur_gray

        mask = np.zeros_like(edge_img) # warped와 같은 크기의 0 행렬 생성
        mask = cv2.fillPoly(mask, self.vertices, fill_color) # vertices 좌표에 fill_color를 가지는 다각형의 mask 생성  
        roi = cv2.bitwise_and(edge_img, mask) # 비트연산을 통해 warped 이미지에 mask 적용
        roi = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)

        return roi
    
    def binarization(self, img):
        threshold,threshold2,threshold3 = 190, 195, 200
        ### HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        _, _, v = cv2.split(hsv)
        hsv_low_white = np.array((0, 0, threshold))
        hsv_high_white = np.array((255, 255, 255))
        hsv_binary = cv2.inRange(hsv, hsv_low_white, hsv_high_white)

        ### HLS color space
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        _, l, _ = cv2.split(hls)
        hls_low_white = np.array((0, threshold2,  0))
        hls_high_white = np.array((255, 255, 255))
        hls_binary = cv2.inRange(hls, hls_low_white, hls_high_white)

        ### R color channel 경계값 설정
        _, _, r = cv2.split(img)
        r_low_white = threshold3
        r_high_white = 255
        r_binary = cv2.inRange(r, r_low_white, r_high_white)        
        combined = np.asarray(r_binary/255, dtype=np.uint8) +  np.asarray(hls_binary/255, dtype=np.uint8) + np.asarray(hsv_binary/255, dtype=np.uint8)
        
        combined[combined < 2] = 0
        combined[combined >= 2] = 255
        return  combined
    
    def polyfit(self, binary, debug):
        # 이진 영상을 3차원 R,G,B 영상으로 변환
        out = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        if binary.max() <= 0: # no white pixels
            return LaneDetector.ERR_NO_WHITE_PIXELS, out, None, None, None, None

        nonzero = binary.nonzero()
        nonzerox = np.array(nonzero[1])
        nonzeroy = np.array(nonzero[0])

        leftx_current = self.half_width - self.margin*4
        rightx_current = self.half_width + self.margin*4

        left_lane_inds = []
        right_lane_inds = []

        # sliding window의 초기 위치 선정
        win_y_low = self.img_height - (1 + 0) * self.window_height
        win_y_high = self.img_height - 0 * self.window_height
        while leftx_current > self.margin:
            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin

            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                            & (nonzerox >= win_xleft_low) & (nonzerox <= win_xleft_high)).nonzero()[0]
            if len(good_left_inds) >  self.minpix:
                left_lane_inds.append(good_left_inds)
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
                break
            else:
                leftx_current -= 2*self.margin

        while rightx_current < self.img_width-self.margin:
            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin
            
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                            & (nonzerox >= win_xright_low) & (nonzerox <= win_xright_high)).nonzero()[0]
            if len(good_right_inds) > self.minpix:
                right_lane_inds.append(good_right_inds)
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
                break
            else:
                rightx_current += 2*self.margin


        # sliding window를 만들어서 위로 올라가며 차선의 후보들을 검색
        left_discontinue_count = 0
        right_discontinue_count = 0
        for window in range(self.nb_windows):
            win_y_low = self.img_height - (1 + window) * self.window_height
            win_y_high = self.img_height - window * self.window_height

            win_xleft_low = leftx_current - self.margin
            win_xleft_high = leftx_current + self.margin

            win_xright_low = rightx_current - self.margin
            win_xright_high = rightx_current + self.margin

            if debug:
                # Sliding window를 그린다.
                cv2.rectangle(out, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),\
                            (0, 255, 0), 2)
                cv2.rectangle(out, (win_xright_low, win_y_low), (win_xright_high, win_y_high),\
                            (0, 255, 0), 2)

            # Sliding window에 포함되는 흰 pixel들의 좌표 index를 저장
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                            & (nonzerox >= win_xleft_low) & (nonzerox <= win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= win_y_high)
                            & (nonzerox >= win_xright_low) & (nonzerox <= win_xright_high)).nonzero()[0]

            # Sliding window에 포함되는 흰 pixel의 수가 최소 기준 이상이면 이것들을 차선에 대한 pixel로 포함
            if len(good_left_inds) >  self.minpix:
                left_lane_inds.append(good_left_inds)
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            else:
                left_discontinue_count+=1

            if len(good_right_inds) > self.minpix:
                right_lane_inds.append(good_right_inds)
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
            else:
                right_discontinue_count+=1

        # sliding window가 끊어지는 횟수가 일정 수준 이상이면 차선 후보에서 취소시킴
        if left_discontinue_count >= self.discontinue_threshold:
            left_lane_inds = []
        if right_discontinue_count >= self.discontinue_threshold:
            right_lane_inds = []


        # 왼쪽과 오른쪽 차선에 대한 pixel들의 x,y 좌표 목록을 만든다.
        left_fit, right_fit = None, None
        left_angle, right_angle = (0,0)
        leftx, rightx = [], []
        if left_lane_inds != []:
            left_lane_inds = np.concatenate(left_lane_inds)
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]

        if right_lane_inds != []:
            right_lane_inds = np.concatenate(right_lane_inds)
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]

        # 찾은 차선 후보 pixel의 총 수가 기준값 이상이고, 왼쪽과 오른쪽 차선의 x좌표 평균이 각각 화면의 왼쪽과 오른쪽에 있다면 차선 후보로 선택
        # 선택된 좌표들에 대해 2차 함수 fitting 수행
        if len(leftx) >= self.min_lane_pts and np.mean(leftx) < self.img_width//2-self.offset:
            left_fit = np.polyfit(lefty, leftx, 2)
            out[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            left_angle = self.get_angle(left_fit)
        if len(rightx) >= self.min_lane_pts and np.mean(rightx) > self.img_width//2+self.offset:
            right_fit = np.polyfit(righty, rightx, 2)
            out[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [255, 0, 255]
            right_angle= self.get_angle(right_fit)

        if left_fit is not None and right_fit is not None:
            # 양쪽 차선에 대한 차선 후보를 모두 찾으면 유효한지 검사
            if self.check_validity(left_fit, right_fit):
                #찾은 두 개의 차선이 유효
                ret = LaneDetector.SUCCESS_POLY_FIT_BOTH
                return ret, out, left_fit, right_fit, left_angle, right_angle
            else:
                # 그렇지 않음
                ret = LaneDetector.ERR_NOT_VALID
                return ret, out, None, None, None, None
        elif left_fit is not None or right_fit is not None:
            # 둘 중 하나의 차선만 찾은 경우 유효성 검사를 하지 않고 이대로 결정
            ret = LaneDetector.SUCCESS_POLY_FIT_ONE
            return ret, out, left_fit, right_fit, left_angle, right_angle
        else:
            # 아무 차선도 찾지 못함
            ret = LaneDetector.ERR_NOT_FOUND
            return ret, out, None, None, None, None
        
    def get_angle(self, fit):
        # 차선의 2차 곡선의 제일 아래와 길이의 75% 위치에서 두 점을 선택해서 두 점이 이루는 각도 계산
        _, poly_y = self.get_poly_points(fit)
        y1 = self.img_height - 1 # Bottom
        y2 = self.img_height - int(len(poly_y)* 0.75)
        x1 = fit[0]  * (y1**2) + fit[1]  * y1 + fit[2]
        x2 = fit[0]  * (y2**2) + fit[1]  * y2 + fit[2]

        angle = np.rad2deg(np.arctan2(y2-y1, x2-x1)) # 라디안을 degree로 변환

        return angle

    def calc_output(self, fit):
        if fit is not None:
            poly_x, poly_y = self.get_poly_points(fit)
            mean_x, mean_y = int(np.mean(poly_x)), int(np.mean(poly_y))

            return mean_x, mean_y

    def get_poly_points(self,fit):
        """ 인식된 차선의 2차 피팅 정보를 x,y 좌표로 변환

        Args:
            fit (numpy.array): 인식된 차선의 2차 피팅 정보

        Return:
            x (numpy.array): 인식된 차선의 x 좌표
            y (numpy.array): 인식되 차선의 y 좌표
        """

        ysize, xsize = self.img_height, self.img_width
        
        plot_y = np.linspace(0, ysize-1, ysize)
        plot_x = fit[0] * plot_y**2 + fit[1] * plot_y + fit[2]
        plot_x = plot_x[(plot_x >= 0) & (plot_x <= xsize - 1)]
        plot_y = np.linspace(ysize - len(plot_x), ysize - 1, len(plot_x))

        x = plot_x.astype(int)
        y = plot_y.astype(int)

        return x, y
   
    def draw(self,img, warped, invM, left_fit, right_fit):
        warp_zero = np.zeros_like(warped[:,:,0]).astype(np.uint8) # warped 이미지 사이즈의 0 numpy.array
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero)) # warp_zero를 3차원의 numpy.array로 stack
        
        mode = 0
        # 인식된 차선의 유무에 따라 차선을 그림
        if left_fit is not None:
            plot_xleft, plot_yleft = self.get_poly_points(left_fit)
            pts_left = np.array([np.transpose(np.vstack([plot_xleft, plot_yleft]))])
            cv2.polylines(color_warp, np.int32([pts_left]), isClosed=False,
                            color=(255, 0, 0), thickness=10)
            mode = 1
        if right_fit is not None:
            plot_xright, plot_yright = self.get_poly_points(right_fit)
            pts_right = np.array([np.flipud(np.transpose(np.vstack([plot_xright, plot_yright])))])
            cv2.polylines(color_warp, np.int32([pts_right]), isClosed=False,
                        color=(255, 0, 255), thickness= 10)
            mode += 2 
 
        if mode == 3:
            leftx,lefty = self.calc_output(left_fit)
            rightx,righty = self.calc_output(right_fit)
            centerx = (leftx+rightx)//2
            cv2.circle(color_warp, (centerx,(lefty+righty)//2), 50, (255,0,255), 30)
        elif mode == 1:
            leftx,lefty = self.calc_output(left_fit)
            centerx = leftx + self.lane_width
            rightx = leftx + self.lane_width*2
            cv2.circle(color_warp, (centerx,lefty), 50, (255,0,0), 30)
        elif mode == 2:
            rightx,righty = self.calc_output(right_fit)
            centerx = rightx - self.lane_width
            leftx = rightx - self.lane_width*2
            cv2.circle(color_warp, (centerx,righty), 50, (255,0,255), 30)

        # 차선 정보를 포함하는 탑-다운뷰 이미지를 원래 이미지로 원근 변환함
        unwarped = cv2.warpPerspective(color_warp, invM, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR) 
        
        lane1=np.where((unwarped[:,:,0]!=0) & (unwarped[:,:,2]==0))
        lane2=np.where((unwarped[:,:,2]!=0) & (unwarped[:,:,0]!=0))
        out = img.copy()

        # 인식된 차선을 원본 이미지에 표시
        out[lane1]=(255,0,0)
        out[lane2]=(0,0,255)

        return out, leftx, centerx, rightx

    def check_validity(self, left_fit, right_fit):
        """ 왼쪽, 오른쪽 차선의 존재 유무 판단

        Args:
            left_fit (numpy.array): 인식된 왼쪽 차선의 2차 피팅 정보
            right_fit (numpy.array): 인식된 오른쪽 차선의 2차 피팅 정보

        Return:
            valid (bool): 왼쪽, 오른쪽 두 차선 모두 존재하면 True, 아니면 False
        """

        valid = True

        if left_fit is None or right_fit is None:
            return False, True, True

        _, poly_yleft = self.get_poly_points(left_fit)
        _, poly_yright = self.get_poly_points(right_fit)

        # 두 선이 서로 다른 세 개의 Y 값에 대해 서로 그럴듯한 거리 안에 있는지 확인
        y1 = self.img_height - 1 # Bottom
        y2 = self.img_height - int(min(len(poly_yleft), len(poly_yright)) * 0.35) # 두 번째와 세 번째의 경우 y1과 사용 가능한 최상위 값 사이의 값을 가져옴
        y3 = self.img_height - int(min(len(poly_yleft), len(poly_yright)) * 0.75)

        # 두 라인의 x값을 계산
        x1l = left_fit[0]  * (y1**2) + left_fit[1]  * y1 + left_fit[2]
        x2l = left_fit[0]  * (y2**2) + left_fit[1]  * y2 + left_fit[2]
        x3l = left_fit[0]  * (y3**2) + left_fit[1]  * y3 + left_fit[2]

        x1r = right_fit[0] * (y1**2) + right_fit[1] * y1 + right_fit[2]
        x2r = right_fit[0] * (y2**2) + right_fit[1] * y2 + right_fit[2]
        x3r = right_fit[0] * (y3**2) + right_fit[1] * y3 + right_fit[2]


        # 두 개의 서로 다른 Y 값에 대해 선 기울기가 유사한지 확인
        # x = Ay**2 + By + C
        # dx/dy = 2Ay + B
        # y1right 
        y1left_dx  = 2 * left_fit[0]  * y1 + left_fit[1]
        y3left_dx  = 2 * left_fit[0]  * y3 + left_fit[1]
        y1right_dx = 2 * right_fit[0] * y1 + right_fit[1]
        y3right_dx = 2 * right_fit[0] * y3 + right_fit[1]

        # L1 norm 계산
        norm1 = abs(y1left_dx - y1right_dx)
        norm2 = abs(y3left_dx - y3right_dx)

        # L1 norm 임계값 설정
        thresh = 0.6 #0.58 
        if (norm1 >= thresh) | (norm2 >= thresh):
                valid = False
       
        return valid