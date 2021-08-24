import numpy as np
import math
from sklearn.cluster import KMeans

import sys
if 'threading' in sys.modules:
    del sys.modules['threading']

def calc_deflection_point(x, y, radian):
    """
    计算旋转radian后的点
    :param x:
    :param y:
    :param radian: 旋转弧度
    :return:
    """
    update_x = x * math.cos(radian) + y * math.sin(radian)
    update_y = -x * math.sin(radian) + y * math.cos(radian)
    return update_x, update_y

def parse_obj(filename, radian, direction):
    """
    get points and faces
        注意，faces中间保存是文件中点的下标
    :param filename:
    :return: np.array的点云 list的面
    """
    f = open(filename)
    pointfile = f.read()
    f.close()
    lines = pointfile.split('\n')
    pointfile = [i.split() for i in lines if 'v ' in i]
    points = []
    for i in range(len(pointfile)):
        point = []
        a = eval(pointfile[i][1])
        b = eval(pointfile[i][2])
        c = eval(pointfile[i][3])
        # 旋转点
        a, b = calc_deflection_point(a, b, radian)
        if direction == 0 or direction == -1:
            x,y,z = a, b, c
        elif direction == 1:
            x,y,z = a, -c, b
        elif direction == 2:
            x,y,z = a, c, -b
        elif direction == 3:
            x,y,z = -c, b, a
        elif direction == 4:
            x,y,z = c, b, -a
        else:
            x,y,z = a, b, c
        point.append(x)
        point.append(y)
        point.append(z)
        points.append(point)
    faces = [i.split() for i in lines if 'f ' in i]
    faces = [[eval(ii) for ii in i[1:]] for i in faces]
    print("点数：", len(points))
    print("面数：", len(faces))
    return np.array(points), faces

def mean_unsim(data_other_same, i_data):
    return np.sqrt(((data_other_same - i_data) ** 2).sum(axis=1)).mean()


def calc_performance(data, result):
    '''计算轮廓系数'''
    result = result.ravel()
    result_set = set(result.tolist())
    result_set = list(result_set)
    index_ls = list(range(data.shape[0]))
    side_ls = []
    for i in index_ls:  # 遍历每一个点，进行每个点的计算
        i_data = data[i, :]
        index_other = index_ls.copy().remove(i)
        data_other = data[index_other, :]
        result_other = result[index_other]
        data_other_same = data_other[result_other == result[i]]
        a_i = np.sqrt(((data_other_same - i_data) ** 2).sum(axis=1)).mean()
        b_i_ls = []
        for j in result_set:
            if j == result[i]:
                continue
            data_other_diff = data_other[result_other == j]
            b_i_t = mean_unsim(data_other_diff, i_data)
            b_i_ls.append(b_i_t)
        b_i = min(b_i_ls)
        side_factor_one = (b_i - a_i) / (max(a_i, b_i))
        side_ls.append(side_factor_one)
    return np.mean(side_ls)


def find_k(points):
    """
    聚类
    :param points:
    :return: 聚类数
    """
    data = points[:, -1].reshape(-1, 1)

    pf_ls = []
    c_num = np.arange(2, 6)
    for k in c_num:
        km = KMeans(k)
        km.fit(data)
        res = km.predict(data).reshape(-1)
        performance = calc_performance(data, res)

        # 判断孤立点群
        isolate_points = False
        label_set = set(km.labels_)
        for item in label_set:
            if km.labels_.tolist().count(item) <= 4:
                isolate_points = True
                break

        if isolate_points:
            pf_ls.append(-1)
        else:
            pf_ls.append(performance)
    #print(pf_ls)
    return c_num[np.argmax(pf_ls)]


def get_cluster_center(points, faces, filename, direction):
    """
    获取聚类中心（精确到后三位）,并重新跟新点云points
    :param points:
    :return:
    """
    k = find_k(points)
    print("k=", k)
    height = points[:, -1].reshape(-1, 1)
    km = KMeans(n_clusters=k)
    km.fit(height)

    high = [round(i[0], 3) for i in km.cluster_centers_]
    # refine高度
    if direction != -1:
        height = [high[i] for i in km.labels_]
        points[:, -1] = height
    high.sort()
    print("聚类中心：", high)
    direction_name = ['d','f','b','l','r']
    if direction == -1:
        refine_file = filename  # 修正方向替换本身的文件，不用重命名其他文件名称
    else:
        refine_file = filename_generator(filename, direction_name[direction])
    write_model(points, faces, refine_file)
    return high, points

def write_model(points, faces, outfile):
    """
    写入模型，保存
    :param points 点云
    :param outfile: 输出的OBJ，可以是全路径或相对路径
    :return:
    """
    file = open(outfile, 'w')
    for p in points:
        # if direction == 0 or direction == -1:
        #     x,y,z = p[0], p[1], p[2]
        # elif direction == 1:
        #     x,y,z = p[0], -p[2], p[1]
        # elif direction == 2:
        #     x,y,z = p[0], p[2], -p[1]
        # elif direction == 3:
        #     x,y,z = -p[2], p[1], p[0]
        # elif direction == 4:
        #     x,y,z = p[2], p[1], -p[0]
        x,y,z = p[0], p[1], p[2]
        file.write('v ' + str(x) + ' ' + str(y) + ' ' + str(z) + '\n')
    for f in faces:
        line = 'f '
        for i in f:
            line = line + str(i) + ' '
        file.write(line + '\n')
    file.close()

def filename_generator(filename, direction='d'):
    """
    文件名称生成器
    :param filename:
    :param direction: d l r f b
    :return:
    """
    file_path_list = filename.split("/")
    refine_file = "/".join(file_path_list[:-1])
    prefix = "/" + file_path_list[len(file_path_list) - 1][:-4] + "_" + direction + ".obj"
    refine_file += prefix
    return refine_file

def calc_distance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
def calc_radian(x1,y1,x2,y2):
    return math.atan((y1-y2)/(x1-x2))

def calc_longest_link(floor_contour_link, points):
    """
    计算最长的link，输出索引tuple
    :param floor_contour_link: 第一层的轮廓线
    :param points:
    :return:
    """
    link_distance = []
    for link in floor_contour_link:
        distance = calc_distance((points[link[0] - 1][0]),
                                 points[link[0] - 1][1],
                                 points[link[1] - 1][0],
                                 points[link[1] - 1][1])
        link_distance.append(distance)
    max_index = link_distance.index(max(link_distance))
    #print("最大距离下标：", max_index)
    #print("最大距离link：", floor_contour_link[max_index])
    return floor_contour_link[max_index]

def run(filename, radian, direction):
    #filename = '/home/mitom/data/obj/single-plat-result.obj'
    direction_list = ['d','f', 'b', 'l', 'r']
    if direction >= 0:
        filename = filename_generator(filename, 's')
    points, faces = parse_obj(filename, radian, direction)
    if direction == -1: # 修正方向而已
        filename = filename_generator(filename, 's')
        write_model(points, faces, filename)
        return 1
    #print(filename, radian)
    height, points = get_cluster_center(points, faces, filename, direction)
    #print(height)
    return 1

def get_rotation(filename):
    points, faces = parse_obj(filename, 0, 0)
    k = find_k(points)
    print("k=", k)
    height = points[:, -1].reshape(-1, 1)
    km = KMeans(n_clusters=k)
    km.fit(height)

    high = [round(i[0], 3) for i in km.cluster_centers_]
    # refine高度
    height = [high[i] for i in km.labels_]
    points[:, -1] = height
    #print(min(high))
    floor_height = min(high)
    #print(points)
    floor_points = []
    for i in range(len(points)):
        if points[i][2] == floor_height:
            floor_points.append(i+1)
    floor_faces_index = []
    for i in range(len(faces)):
        if set(faces[i]) <= set(floor_points):
            floor_faces_index.append(i)
    # print(floor_faces_index)

    # 所有面的连通关系
    floor_faces_set = []
    for i in floor_faces_index:
        floor_faces_set.append(faces[i])
    connected_list = []
    for i in range(len(floor_faces_set)):
        for j in range(len(floor_faces_set[i])):
            link = []
            if j+1 <= (len(floor_faces_set[i])-1):
                link.append(floor_faces_set[i][j])
                link.append(floor_faces_set[i][j+1])
                connected_list.append(link)
        connected_list.append([floor_faces_set[i][0], floor_faces_set[i][len(floor_faces_set[i]) - 1]])
    # print(connected_list)
    longest_link = calc_longest_link(connected_list, points)

    radian = calc_radian(points[longest_link[0]-1][0],
                         points[longest_link[0]-1][1],
                         points[longest_link[1]-1][0],
                         points[longest_link[1]-1][1])
    print("偏转角度：", radian)
    return radian

def main(filename):
    rotation = get_rotation(filename)
    # 生成正向高度真实的obj文件
    run(filename, rotation, -1)
    for i in range(5):
        run(filename, 0, i) # 0.22285433253600082
    return 1


# if __name__ == "__main__":
#     filename = '/home/mitom/data/obj/single-plat-result.obj'
#     main(filename)