import numpy as np
from sklearn.cluster import KMeans

import sys
if 'threading' in sys.modules:
    del sys.modules['threading']

def get_pc(filename):
    """
    获取点云
    :param filename:
    :return:
    """
    f = open(filename)
    points = f.read()
    f.close()
    points = points.split('\n')
    points = [i.split() for i in points if 'v ' in i]
    points = [[eval(ii) for ii in i[1:]] for i in points]
    # print(points)
    return np.array(points)


def parse_obj(filename):
    """
    get points and faces
        注意，faces中间保存是文件中点的下标
    :param filename:
    :return: np.array的点云 list的面
    """
    f = open(filename)
    points = f.read()
    f.close()
    lines = points.split('\n')
    points = [i.split() for i in lines if 'v ' in i]
    points = [[eval(ii) for ii in i[1:]] for i in points]

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


def get_cluster_center(points, faces, filename):
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
    height = [high[i] for i in km.labels_]
    points[:, -1] = height
    high.sort()
    print("聚类中心：", high)
    refine_file = filename_generator(filename, 'd')
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
        file.write('v ' + str(p[0]) + ' ' + str(p[1]) + ' ' + str(p[2]) + '\n')
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
    prefix = "/" + file_path_list[len(file_path_list) - 1][:-4] + "_refine_" + direction + ".obj"
    refine_file += prefix
    return refine_file

def run(filename):
    #filename = '/home/mitom/data/obj/single-plat-result.obj'
    points, faces = parse_obj(filename)
    height, points = get_cluster_center(points, faces, filename)
    print(height)
    return 1

# if __name__ == "__main__":
#     run()
