# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: finally를 사용하는 프로그램
def main():
    print("first line")
    while True:
        try:
            print("try")
            # raise
            break
            print("after break")
        except Exception as e:
            print("except", e)
        finally:
            print("finally")
        print("end of while")
    print("end of main")


if __name__ == "__main__":
    main()
