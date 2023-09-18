# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 리스트의 메소드를 사용하는 프로그램
import datetime


def main():
    ptime = datetime.datetime.now()
    list_a = [0, 1, 2, 3, 4, 5, 6]
    list_b = ["a", "b", "c", "d", "e", "f"]
    del list_a[0]
    del list_b[2]
    print(list_a)
    print(list_b)

    _ = list_a.pop()  # 리턴 값이 있음.
    print(list_a)
    list_a.pop(3)
    print(list_a)
    list_a.append(ptime)
    print(list_a)
    del ptime  # 객체를 삭제하기.
    # print(ptime)
    print(list_a[4])
    del list_a[4]  # 리스트안의 요소를 삭제.
    print(list_a)

    list_b.remove("e")  # 요소를 찾아서 삭제함.
    print(list_b)

    # list_b.clear()
    # list_b = []
    # print(list_b)

    if "a" in list_b:
        print("a 는 list_b 안에 있습니다.")


if __name__ == "__main__":
    main()
