# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 파일을 읽고 쓰는 프로그램
def main():
    # f = open('data/text.txt', 'w')
    # f.write('Hello Python Programming...!')
    # f.close()

    with open("data/text.txt", "a") as f:
        f.write("Hello Python Programming... this is with open!\n")


if __name__ == "__main__":
    main()
