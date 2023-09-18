# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 제너레이터를 사용하는 프로그램
def test():
    print("A")
    yield 1
    print("B")
    yield 2
    print("C")
    yield 3


def main():
    output = test()
    # print('D')
    # a = next(output)
    # print(a)

    # print('E')
    # b = next(output)
    # print(b)

    # c = next(output)
    # print(c)

    for i in output:
        print(i)


if __name__ == "__main__":
    main()
