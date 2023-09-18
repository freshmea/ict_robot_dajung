# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: 제너레이터를 사용하는 프로그램
def test():
    print("함수가 호출되었습니다.")
    yield "test"


def main():
    # 실행이 안되는 예제.
    print("A 지점 통과")
    test()

    print("B 지점 통과")
    test()
    print(test())


if __name__ == "__main__":
    main()
