import test_package.package_a as a
import test_package.package_b as b
# import sys


def main():
    # print(sys.path)
    print(a.variable_a)
    a.a_test()

    print(b.variable_b)
    b.b_test()

if __name__ == '__main__':
    main()