# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: flatten 함수를 구현하는 프로그램
output = list()


def flatten(data: list):
    global output
    li = data.pop(0)
    # for li in data:
    #     output.extend(li)
    output.extend(li)
    if not data:
        return output
    else:
        return flatten(data)


def main():
    example = [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12], [13, 14, 15]]
    print("원본:", example)
    print("변환:", flatten(example))


if __name__ == "__main__":
    main()
