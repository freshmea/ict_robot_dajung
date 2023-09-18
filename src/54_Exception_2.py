# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 사용자 정의 예외를 발생시키는 프로그램
class Myerror(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)
        self.args = ["이것은 내가 만든 에러 입니다."]


def main():
    number = input("정수 입력")
    number = int(number)
    try:
        if number < 0:
            raise Myerror
        else:
            raise NotImplementedError
    except NotImplementedError:
        print("아직 구현 되지 않음")
    except Myerror as e:
        print(e)
        print("내가 만든 에러객체")


if __name__ == "__main__":
    main()
