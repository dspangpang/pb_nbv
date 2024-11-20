# 用于计算平行夹爪与moveit控制的线性映射关系

import numpy as np

def calculate_slope_and_intercept(point1, point2):
    # 计算斜率
    slope = (point2[1] - point1[1]) / (point2[0] - point1[0])

    # 计算截距
    intercept = point1[1] - slope * point1[0]

    return slope, intercept


# 主函数
if __name__ == '__main__':
    
    point_1 = (0, 0.65)
    point_2 = (0.095, 0)
    
    slope, intercept = calculate_slope_and_intercept(point_1, point_2)
    print("斜率: ", slope)
    print("截距: ", intercept)