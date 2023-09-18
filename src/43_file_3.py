# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 파일 입출력 프로그램
def main():
    # with open('data/text.txt', 'r') as f:
    #     data = f.read()
    # print(data)

    # with open('data/text.txt', 'r') as f:
    #     data = ' '
    #     while(data):
    #         data = f.readline()
    #         print(data, end='')

    with open("data/text.txt", "r") as f:
        data = f.readlines()
        print(data)
        for str in data:
            print(str, end="")


if __name__ == "__main__":
    main()
