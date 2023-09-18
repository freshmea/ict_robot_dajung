# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: finally를 사용하는 프로그램
def test():
    print("first line")
    try:
        print("try")
        # raise
        return
        print("after return")
    except Exception as e:
        print("except", e)
    else:
        print("else")
    finally:
        print("finally")
    print("end of test")


def main():
    test()


if __name__ == "__main__":
    main()
