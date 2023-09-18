# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 가변 매개변수와 가변 키워드 매개변수를 사용하여 출력하는 프로그램
def print_n_times(n, *args, **keyargs):
    for i in range(n):
        print(args)

    print(keyargs)
    print(type(keyargs))
    for i in keyargs:
        print(keyargs[i])
    return


def main():
    print_n_times(2, "a", "b", "c", d=1, e=2, f=3)


if __name__ == "__main__":
    main()
