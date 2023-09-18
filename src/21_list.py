# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 리스트를 사용하는 프로그램
import datetime


def main():
    # 리스트 변수 만들기1
    list_a = []
    print(list_a)
    # 리스트 변수 만들기2
    list_b = list()
    print(list_b)
    print(type(list_b))

    ptime = datetime.datetime.now()
    print(type(ptime))
    list_c = [1, 2, 3.3, "choi", "su", "gil", ptime]

    print(list_c[3])
    print(list_c[-1])

    # 이중 리스트
    list_d = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    print(list_d[1])
    print(list_d[1][2])
    # print(list_d[3])  # out of index


if __name__ == "__main__":
    main()
