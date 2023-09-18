# author: choi sugil
# date: 2023.09.19 version: 1.0.0 license: MIT brief: keyward
# description: sys 모듈을 사용하는 프로그램
import sys


def main():
    for value in sys.argv:
        print(value)
    # if sys.argv[1]  == 'default':
    #     print('this is default mode')
    # if sys.argv[1] == 'active':
    #     print('this is active mode')
    sys.exit()
    print(
        sys.version, sys.version_info, sys.copyright, sys.api_version, sys.getprofile()
    )


if __name__ == "__main__":
    main()
