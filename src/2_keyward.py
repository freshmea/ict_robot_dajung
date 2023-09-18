# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 키워드 출력하기
import keyword


def main():
    print(keyword.kwlist)
    for i in keyword.kwlist:
        print(i)
    print(len(keyword.kwlist))


if __name__ == "__main__":
    main()
