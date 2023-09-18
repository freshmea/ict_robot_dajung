# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 가변 매개변수를 사용하여 함수를 호출하는 프로그램
def print_n_times(n, *value):
    print(type(value))
    for i in range(n):
        for v in value:
            print(v)
        print()


def main():
    print_n_times(3, "abc", "def", "ghi", "jk")
    tu = tuple()
    li = []
    dic = {}
    print(li, dic)
    tu = (1,)
    print(type(tu))


if __name__ == "__main__":
    main()
