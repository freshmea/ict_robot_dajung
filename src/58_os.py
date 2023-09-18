# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: os 모듈을 사용하는 프로그램
import os


def main():
    print(os.name)
    print(os.getcwd())
    print(os.listdir())

    os.mkdir("source")
    os.rmdir("source")
    with open("original.txt", "w") as f:
        f.write("hello")
    os.rename("original.txt", "new.txt")
    os.remove("new.txt")

    os.system("ls")
    os.system("ls")
    dir = "/home/aa/ict_robot_dajung/src"
    os.chdir(dir)
    os.mkdir("source")
    os.rmdir("source")
    print(os.getcwd())
    os.system("ls")


if __name__ == "__main__":
    main()
