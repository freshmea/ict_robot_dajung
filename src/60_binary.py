# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 이미지를 다운로드하는 프로그램
from urllib import request


def main():
    target = request.urlopen("http://www.hanbit.co.kr/images/common/logo_hanbit.png")
    output = target.read()
    with open("data/output.png", "wb") as f:
        f.write(output)


if __name__ == "__main__":
    main()
